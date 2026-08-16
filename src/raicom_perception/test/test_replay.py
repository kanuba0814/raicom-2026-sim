import json
from pathlib import Path

import pytest

from raicom_perception import (
    ObservationState,
    ReplayConfig,
    ReplayDetection,
    ReplayDetector,
    ReplayFrame,
    UnavailableLiveCameraDetector,
)


def config(**overrides):
    values = {
        "class_mapping": {"raw_a": "material_a"},
        "minimum_confidence": 0.7,
        "minimum_stability": 1.0,
        "required_consecutive_frames": 2,
        "maximum_frame_age_s": 0.5,
        "model_path": "candidate.onnx",
        "model_sha256": "a" * 64,
        "model_verification": "SIM_TESTED",
    }
    values.update(overrides)
    return ReplayConfig(**values)


def frame(frame_id="frame", timestamp=10.0, detections=()):
    return ReplayFrame(frame_id, timestamp, tuple(detections))


def detection(raw_class="raw_a", confidence=0.9, bbox=(10, 20, 30, 40)):
    return ReplayDetection(raw_class, confidence, bbox)


def test_replay_maps_classes_and_applies_temporal_gate_without_arm_coordinates():
    detector = ReplayDetector(config(), now=lambda: 10.1)
    first = detector.detect(frame("first", detections=(detection(),)))
    result = detector.detect(frame("second", detections=(detection(),)))

    assert first.state == ObservationState.NOT_DETECTED
    assert result.state == ObservationState.DETECTED
    target = result.targets[0]
    assert target.class_name == "material_a"
    assert target.image_center_px == (20.0, 30.0)
    assert target.stability == 1.0
    assert not target.inside_pick_roi
    assert not hasattr(target, "arm_coordinates")


def test_confidence_and_stability_gates_return_not_detected():
    detector = ReplayDetector(config(required_consecutive_frames=1, minimum_stability=1.0), now=lambda: 10.0)
    result = detector.detect(frame(detections=(detection(confidence=0.69),)))
    assert result.state == ObservationState.NOT_DETECTED
    assert result.reason == "NO_CONFIDENT_STABLE_MAPPED_TARGET"


def test_unknown_frame_breaks_temporal_stability():
    detector = ReplayDetector(config(), now=lambda: 10.0)
    detector.detect(frame("first", detections=(detection(),)))
    assert detector.detect(None).state == ObservationState.UNKNOWN
    assert detector.detect(frame("second", detections=(detection(),))).state == ObservationState.NOT_DETECTED


@pytest.mark.parametrize(
    "replay_frame, expected_reason",
    [
        (None, "FRAME_MISSING"),
        (ReplayFrame("camera", 10.0, camera_available=False), "CAMERA_UNAVAILABLE"),
        (ReplayFrame("bad", 10.0, malformed=True), "FRAME_MALFORMED"),
        (ReplayFrame("stale", 8.0), "FRAME_STALE"),
        (ReplayFrame("unmapped", 10.0, (detection(raw_class="other"),)), "UNMAPPED_CLASS:other"),
        (ReplayFrame("bad-output", 10.0, (object(),)), "DETECTION_MALFORMED"),
    ],
)
def test_bad_frame_conditions_are_unknown(replay_frame, expected_reason):
    detector = ReplayDetector(config(required_consecutive_frames=1), now=lambda: 10.0)
    result = detector.detect(replay_frame)
    assert result.state == ObservationState.UNKNOWN
    assert result.reason == expected_reason


def test_missing_recorded_output_and_model_are_explicitly_unavailable():
    detector = ReplayDetector(config(), now=lambda: 10.0)
    assert detector.detect(ReplayFrame("missing-output", 10.0, None)).state == ObservationState.UNAVAILABLE
    unavailable_model = ReplayDetector(config(), model_available=False)
    assert unavailable_model.detect(frame()).reason == "MODEL_UNAVAILABLE"
    assert UnavailableLiveCameraDetector().detect(object()).reason == "LIVE_CAMERA_CONTRACT_UNVERIFIED"


@pytest.mark.parametrize("nested_value", [{"nested": "value"}, ["nested"], object(), True])
def test_direct_config_rejects_nested_or_nonstring_mapping_values(nested_value):
    with pytest.raises((TypeError, ValueError)):
        config(class_mapping={"raw_a": nested_value})


def test_direct_config_copies_mapping_and_rejects_unsafe_provenance():
    source = {"raw_a": "material_a"}
    copied = config(class_mapping=source)
    source["raw_a"] = "mutated"
    assert copied.class_mapping["raw_a"] == "material_a"
    for overrides in (
        {"model_path": "/host/model.onnx"}, {"model_path": "../model.onnx"},
        {"model_path": "x" * 513}, {"model_path": "bad\npath"},
        {"model_sha256": "A" * 64}, {"minimum_confidence": True},
        {"maximum_frame_age_s": float("inf")},
    ):
        with pytest.raises((TypeError, ValueError)):
            config(**overrides)


def test_config_records_provenance_and_rejects_field_validated_claim(tmp_path):
    metadata_path = tmp_path / "fixture.json"
    metadata_path.write_text(json.dumps({
        "class_mapping": {"raw_a": "material_a"},
        "model_sha256": "b" * 64,
        "model_verification": "SIM_TESTED",
    }))
    loaded = ReplayConfig.from_json(metadata_path)
    provenance = ReplayDetector(loaded).provenance
    assert provenance["model_sha256"] == "b" * 64
    assert provenance["verification"] == "SIM_TESTED"
    with pytest.raises(ValueError, match="FIELD_VALIDATED"):
        config(model_verification="FIELD_VALIDATED")


def test_sim_composition_adapter_requires_explicit_replay_frame_and_preserves_unknown() -> None:
    from raicom_perception import SimReplayPerceptionAdapter
    detector = ReplayDetector(ReplayConfig({"block": "block"}))
    adapter = SimReplayPerceptionAdapter(detector)
    frame = ReplayFrame("camera-off", 1.0, camera_available=False)
    result = adapter.perceive(frame)
    assert result.state == ObservationState.UNKNOWN
    assert result.reason == "CAMERA_UNAVAILABLE"
