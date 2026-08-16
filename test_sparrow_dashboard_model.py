from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).parent
MODELS = ROOT / "models/models"
AIRFRAME = ROOT / "patches/px4-airframes/4020_gz_sparrow_gimbal"


def test_native_sparrow_model_chain_and_dashboard_payload_are_present() -> None:
    for name in ("sparrow_base", "sparrow", "sparrow_gimbal_core", "sparrow_gimbal"):
        assert (MODELS / name / "model.sdf").is_file()

    vehicle = (MODELS / "sparrow/model.sdf").read_text(encoding="utf-8")
    wrapper = (MODELS / "sparrow_gimbal/model.sdf").read_text(encoding="utf-8")
    assert vehicle.count("<motorNumber>") == 4
    assert vehicle.count("<maxRotVelocity>1400.0</maxRotVelocity>") == 4
    assert "model://sparrow_gimbal_core" in wrapper
    assert "<topic>/sparrow_gimbal/front_lidar</topic>" in wrapper


def test_sparrow_airframe_pins_rotors_tune_and_acceleration_contract() -> None:
    text = AIRFRAME.read_text(encoding="utf-8")
    for value in ("CA_ROTOR0_KM  0.016", "CA_ROTOR2_KM -0.016"):
        assert value in text
    assert "SIM_GZ_EC_MAX4 1400" in text
    assert "MPC_THR_HOVER 0.66" in text
    assert "MPC_ACC_HOR_MAX 4.0" in text
    assert "MC_ROLLRATE_P 0.167565" in text


def test_sparrow_runtime_defaults_keep_flight_authority_off() -> None:
    environment = (ROOT / ".env.example").read_text(encoding="utf-8")
    assert "SWARM_PX4_SYS_AUTOSTART=4020" in environment
    assert "SWARM_PX4_SIM_MODEL=gz_sparrow_gimbal" in environment
    assert "SWARM_GAZEBO_MODEL_UAV_02=sparrow_gimbal_1" in environment
    assert "SWARM_CBF_RL_MODE=off" in environment
