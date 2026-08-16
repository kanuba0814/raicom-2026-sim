"""Append-only run event journal and deterministic accepted-score projection."""
from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256
import errno
import fcntl
import json
import math
import os
from pathlib import Path
import stat
from typing import Any, Iterable, List, Mapping, Optional, Set, Tuple, Union


class JournalIntegrityError(ValueError):
    """Raised for malformed, truncated, or hash-chain-invalid journal data."""


class JournalConflictError(JournalIntegrityError):
    """Raised when a command ID is reused for a different immutable event."""


@dataclass(frozen=True)
class EventRecord:
    """An immutable journal record; command_id is absent in legacy records."""

    sequence: int
    timestamp_s: float
    event_type: str
    payload: Mapping[str, Any]
    previous_hash: str
    event_hash: str
    command_id: Optional[str] = None


# Retain the original public name for consumers of journals created before command IDs.
RunEvent = EventRecord


class EventJournal:
    """Append canonical JSONL below a trusted root using a relative path only."""

    GENESIS_HASH = "0" * 64
    MAX_JOURNAL_BYTES = 4 * 1024 * 1024
    MAX_JOURNAL_EVENTS = 10_000
    MAX_COMMAND_ID_LENGTH = 128
    MAX_EVENT_TYPE_LENGTH = 128
    MAX_PAYLOAD_BYTES = 64 * 1024

    def __init__(self, artifact_root: Union[Path, str], journal_relative_path: Union[Path, str]) -> None:
        self.artifact_root = _trusted_artifact_root(artifact_root)
        self.relative_path = _relative_journal_path(journal_relative_path)
        # This is retained for diagnostics and backwards-compatible callers; all I/O
        # below is descriptor-relative and never reopens this unchecked pathname.
        self.path = self.artifact_root.joinpath(self.relative_path)
        file_descriptor = _open_journal_descriptor(
            self.artifact_root, self.relative_path, create_parents=True, flags=os.O_RDONLY,
        )
        try:
            events = _read_events_from_descriptor(file_descriptor) if file_descriptor is not None else []
        finally:
            if file_descriptor is not None:
                os.close(file_descriptor)
        self._sequence = len(events)
        self._previous_hash = events[-1].event_hash if events else self.GENESIS_HASH

    def append(self, event_type: str, payload: Mapping[str, Any], timestamp_s: float) -> EventRecord:
        """Append a non-idempotent legacy-compatible event."""
        return self._append(event_type, payload, timestamp_s, command_id=None)

    def append_once(
        self, command_id: str, event_type: str, payload: Mapping[str, Any], timestamp_s: float,
    ) -> EventRecord:
        """Atomically append an event once for ``command_id``.

        A retry with the same command ID, type, and canonical payload returns the
        original record without writing. Its timestamp is deliberately ignored:
        callers commonly generate a fresh retry timestamp after an uncertain
        response. Reusing an ID for a different type or payload is a conflict.
        """
        _validate_command_id(command_id)
        return self._append(event_type, payload, timestamp_s, command_id=command_id)

    def _append(
        self, event_type: str, payload: Mapping[str, Any], timestamp_s: float, command_id: Optional[str],
    ) -> EventRecord:
        canonical_event_type, canonical_payload, canonical_timestamp = _validate_event_inputs(
            event_type, payload, timestamp_s,
        )
        flags = os.O_RDWR | os.O_APPEND | os.O_CREAT
        file_descriptor = _open_journal_descriptor(
            self.artifact_root, self.relative_path, create_parents=True, flags=flags, mode=0o600,
        )
        assert file_descriptor is not None
        try:
            if not stat.S_ISREG(os.fstat(file_descriptor).st_mode):
                raise JournalIntegrityError("journal path is not a regular file")
            fcntl.flock(file_descriptor, fcntl.LOCK_EX)
            try:
                # Validation, duplicate detection, and append share one lock to prevent TOCTOU.
                events = _read_events_from_descriptor(file_descriptor)
                if command_id is not None:
                    for event in events:
                        if event.command_id != command_id:
                            continue
                        if _event_identity(event.event_type, event.payload) == _event_identity(
                            canonical_event_type, canonical_payload,
                        ):
                            return event
                        raise JournalConflictError("command_id conflicts with existing event")
                if len(events) >= self.MAX_JOURNAL_EVENTS:
                    raise JournalIntegrityError("journal exceeds bounded event count")
                sequence = len(events) + 1
                previous_hash = events[-1].event_hash if events else self.GENESIS_HASH
                body = _event_body(
                    sequence, canonical_timestamp, canonical_event_type, canonical_payload, previous_hash, command_id,
                )
                event_hash = _hash_body(body)
                record = dict(body, event_hash=event_hash)
                line = (_canonical_json(record) + "\n").encode("utf-8")
                if len(line) > self.MAX_JOURNAL_BYTES or os.fstat(file_descriptor).st_size + len(line) > self.MAX_JOURNAL_BYTES:
                    raise ValueError("journal event exceeds bounded journal capacity")
                written = os.write(file_descriptor, line)
                if written != len(line):
                    raise OSError("incomplete journal append")
                os.fsync(file_descriptor)
            finally:
                fcntl.flock(file_descriptor, fcntl.LOCK_UN)
        finally:
            os.close(file_descriptor)

        event = _parse_event(record)
        self._sequence = sequence
        self._previous_hash = event_hash
        return event

    @classmethod
    def read(cls, artifact_root: Union[Path, str], journal_relative_path: Union[Path, str]) -> List[EventRecord]:
        """Read only a journal that remains confined to its trusted root."""
        root = _trusted_artifact_root(artifact_root)
        relative_path = _relative_journal_path(journal_relative_path)
        file_descriptor = _open_journal_descriptor(root, relative_path, create_parents=False, flags=os.O_RDONLY)
        try:
            return _read_events_from_descriptor(file_descriptor) if file_descriptor is not None else []
        finally:
            if file_descriptor is not None:
                os.close(file_descriptor)


class AcceptedScoreMirror:
    """Projection that changes only for unique explicit referee acceptance events."""

    ACCEPTANCE_EVENT = "REFEREE_ACCEPTED"

    def __init__(self) -> None:
        self._accepted_task_ids: Set[str] = set()
        self._score = 0

    @property
    def score(self) -> int:
        return self._score

    def apply(self, event: EventRecord) -> bool:
        if event.event_type != self.ACCEPTANCE_EVENT:
            return False
        task_id = event.payload.get("task_id")
        score = event.payload.get("score")
        if not isinstance(task_id, str) or not task_id or isinstance(score, bool) or not isinstance(score, int) or score < 0:
            raise ValueError("acceptance event requires non-empty task_id and non-negative integer score")
        if task_id in self._accepted_task_ids:
            return False
        self._accepted_task_ids.add(task_id)
        self._score += score
        return True

    @classmethod
    def rebuild(cls, events: Iterable[EventRecord]) -> "AcceptedScoreMirror":
        mirror = cls()
        for event in events:
            mirror.apply(event)
        return mirror


def _trusted_artifact_root(artifact_root: Union[Path, str]) -> Path:
    root = Path(artifact_root)
    if not root.is_absolute() or root.is_symlink() or not root.is_dir():
        raise ValueError("artifact_root must be an existing, non-symlink absolute directory")
    return root.resolve(strict=True)


def _relative_journal_path(journal_relative_path: Union[Path, str]) -> Path:
    path = Path(journal_relative_path)
    if path.is_absolute() or not path.name or any(part in ("", ".", "..") for part in path.parts):
        raise ValueError("journal path must be a non-empty relative path without traversal")
    return path


def _open_journal_descriptor(
    root: Path, relative_path: Path, create_parents: bool, flags: int, mode: int = 0o600,
) -> Optional[int]:
    """Open a journal beneath ``root`` without resolving untrusted path components."""
    directory_flags = os.O_RDONLY | os.O_DIRECTORY
    nofollow = getattr(os, "O_NOFOLLOW", 0)
    root_descriptor = os.open(str(root), directory_flags | nofollow)
    parent_descriptor = root_descriptor
    try:
        if not stat.S_ISDIR(os.fstat(parent_descriptor).st_mode):
            raise ValueError("artifact_root must be a directory")
        for part in relative_path.parts[:-1]:
            try:
                child_descriptor = os.open(part, directory_flags | nofollow, dir_fd=parent_descriptor)
            except FileNotFoundError:
                if not create_parents:
                    return None
                try:
                    os.mkdir(part, 0o777, dir_fd=parent_descriptor)
                except FileExistsError:
                    # A concurrent creator won. Re-open below, still without following it.
                    pass
                try:
                    child_descriptor = os.open(part, directory_flags | nofollow, dir_fd=parent_descriptor)
                except OSError as exc:
                    raise ValueError("journal path must not traverse a symlink") from exc
            except OSError as exc:
                raise ValueError("journal path must not traverse a symlink") from exc
            try:
                os.close(parent_descriptor)
            except BaseException:
                os.close(child_descriptor)
                raise
            parent_descriptor = child_descriptor
        try:
            return os.open(relative_path.name, flags | nofollow, mode, dir_fd=parent_descriptor)
        except FileNotFoundError:
            if flags & os.O_CREAT:
                raise
            return None
        except OSError as exc:
            if exc.errno == errno.ELOOP:
                raise ValueError("journal path must not be a symlink") from exc
            raise
    finally:
        os.close(parent_descriptor)


def _event_identity(event_type: str, payload: Mapping[str, Any]) -> str:
    """Canonical serialized identity preserves JSON scalar types (e.g. 1 != 1.0)."""
    return _canonical_json({"event_type": event_type, "payload": payload})


def _read_events_from_descriptor(file_descriptor: int) -> List[EventRecord]:
    descriptor_stat = os.fstat(file_descriptor)
    if not stat.S_ISREG(descriptor_stat.st_mode):
        raise JournalIntegrityError("journal path is not a regular file")
    if descriptor_stat.st_size > EventJournal.MAX_JOURNAL_BYTES:
        raise JournalIntegrityError("journal exceeds bounded capacity")
    os.lseek(file_descriptor, 0, os.SEEK_SET)
    with os.fdopen(os.dup(file_descriptor), "r", encoding="utf-8") as journal:
        return _parse_journal_lines(journal)


def _parse_journal_lines(lines: Iterable[str]) -> List[EventRecord]:
    events: List[EventRecord] = []
    previous_hash = EventJournal.GENESIS_HASH
    for line_number, line in enumerate(lines, 1):
        if line_number > EventJournal.MAX_JOURNAL_EVENTS:
            raise JournalIntegrityError("journal exceeds bounded event count")
        if not line.endswith("\n"):
            raise JournalIntegrityError("journal is truncated at line {0}".format(line_number))
        try:
            record = json.loads(line)
            event = _parse_event(record)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            raise JournalIntegrityError("invalid event at line {0}".format(line_number)) from exc
        if event.sequence != line_number or event.previous_hash != previous_hash:
            raise JournalIntegrityError("broken event sequence or chain at line {0}".format(line_number))
        body = _event_body(
            event.sequence, event.timestamp_s, event.event_type, event.payload, event.previous_hash, event.command_id,
        )
        if event.event_hash != _hash_body(body):
            raise JournalIntegrityError("event hash mismatch at line {0}".format(line_number))
        events.append(event)
        previous_hash = event.event_hash
    return events


def _validate_command_id(command_id: str) -> None:
    if not isinstance(command_id, str) or not command_id or len(command_id) > EventJournal.MAX_COMMAND_ID_LENGTH:
        raise ValueError("command_id must be a non-empty string within the maximum length")
    if command_id != command_id.strip() or any(ord(character) < 32 for character in command_id):
        raise ValueError("command_id must not contain surrounding whitespace or control characters")


def _validate_event_inputs(event_type: str, payload: Mapping[str, Any], timestamp_s: float) -> Tuple[str, Mapping[str, Any], float]:
    if not isinstance(event_type, str) or not event_type.strip() or len(event_type) > EventJournal.MAX_EVENT_TYPE_LENGTH:
        raise ValueError("event type must be a non-empty bounded string")
    if event_type != event_type.strip() or any(ord(character) < 32 for character in event_type):
        raise ValueError("event type must not contain surrounding whitespace or control characters")
    if isinstance(timestamp_s, bool) or not isinstance(timestamp_s, (int, float)) or not math.isfinite(timestamp_s):
        raise ValueError("event timestamp must be finite")
    if not isinstance(payload, Mapping):
        raise ValueError("event payload must be a mapping")
    canonical_payload = _json_copy(payload)
    if len(_canonical_json(canonical_payload).encode("utf-8")) > EventJournal.MAX_PAYLOAD_BYTES:
        raise ValueError("event payload exceeds bounded capacity")
    return event_type, canonical_payload, timestamp_s


def _event_body(sequence: int, timestamp_s: float, event_type: str, payload: Mapping[str, Any], previous_hash: str, command_id: Optional[str]) -> Mapping[str, Any]:
    body = {
        "sequence": sequence,
        "timestamp_s": timestamp_s,
        "event_type": event_type,
        "payload": _json_copy(payload),
        "previous_hash": previous_hash,
    }
    if command_id is not None:
        body["command_id"] = command_id
    return body


def _canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False)


def _json_copy(value: Any) -> Any:
    return json.loads(_canonical_json(value))


def _hash_body(body: Mapping[str, Any]) -> str:
    return sha256(_canonical_json(body).encode("utf-8")).hexdigest()


def _parse_event(record: Mapping[str, Any]) -> EventRecord:
    legacy_fields = {"sequence", "timestamp_s", "event_type", "payload", "previous_hash", "event_hash"}
    command_fields = legacy_fields | {"command_id"}
    if set(record) not in (legacy_fields, command_fields) or not isinstance(record["sequence"], int) or record["sequence"] < 1:
        raise ValueError("invalid event fields")
    if isinstance(record["timestamp_s"], bool) or not isinstance(record["timestamp_s"], (int, float)) or not math.isfinite(record["timestamp_s"]):
        raise ValueError("invalid timestamp")
    if not isinstance(record["event_type"], str) or not record["event_type"]:
        raise ValueError("invalid event type")
    if not isinstance(record["payload"], dict):
        raise ValueError("invalid payload")
    if not all(isinstance(record[key], str) and len(record[key]) == 64 for key in ("previous_hash", "event_hash")):
        raise ValueError("invalid hash")
    command_id = record.get("command_id")
    if command_id is not None:
        _validate_command_id(command_id)
        _validate_event_inputs(record["event_type"], record["payload"], record["timestamp_s"])
    return EventRecord(
        sequence=record["sequence"], timestamp_s=record["timestamp_s"], event_type=record["event_type"],
        payload=_json_copy(record["payload"]), previous_hash=record["previous_hash"], event_hash=record["event_hash"],
        command_id=command_id,
    )
