"""Read-only readiness evaluation for the three-drone SITL pipeline."""

import math

from docker.deployment_command_gate import accepted_velocity
from docker.deployment_settings import DeploymentSettings


def evaluate_readiness(
    manifest,
    scenario_hash,
    now,
    status,
    frames,
    agents,
    poses,
    plans,
    *,
    require_mission=False,
):
    settings = DeploymentSettings.from_manifest(manifest)
    reasons = []

    def fresh(packet, age):
        try:
            stamp = float(packet["timestamp"])
            return (
                packet["scenario_hash"] == scenario_hash
                and math.isfinite(stamp)
                and 0 <= now - stamp <= age
            )
        except (KeyError, TypeError, ValueError):
            return False

    retention = manifest.get("estimator", {}).get("retention", {})
    replacement_active = (
        retention.get("policy") == "paper_epistemic"
        and retention.get("on_full") == "information"
        and retention.get("max_total_samples") is not None
    )

    def fidelity_contributed(fidelity):
        if status.get(f"{fidelity}_sample_count", 0) > 0:
            return True
        history = status.get("sample_filter", {})
        return bool(
            replacement_active
            and isinstance(history, dict)
            and history.get("enabled") is True
            and history.get(f"accepted_{fidelity}", 0) > 0
        )

    if not fresh(status, 2.0):
        reasons.append("central status missing, stale or mismatched")
    elif (
        status.get("posterior_version", 0) < 1
        or status.get("low_sample_count", 0) + status.get("high_sample_count", 0) < 1
        or not fidelity_contributed("low")
        or not fidelity_contributed("high")
    ):
        reasons.append("central estimator needs both fidelities and a valid posterior")
    offsets = frames.get("offsets", {}) if isinstance(frames, dict) else {}
    frame_session = frames.get("session") if isinstance(frames, dict) else None
    if (
        not fresh(frames, 1.0)
        or not math.isfinite(frames.get("expires_at", float("nan")))
        or frames["expires_at"] <= now
    ):
        reasons.append("frame offsets missing, stale or expired")
        offsets = {}
    vehicles = {}
    for vehicle in manifest["vehicles"]:
        vid = vehicle["id"]
        agent, pose, plan = agents.get(vid), poses.get(vid), plans.get(vid)
        if frame_session is not None and any(
            not isinstance(packet, dict) or packet.get("frame_session") != frame_session
            for packet in (agent, pose, plan)
        ):
            reasons.append(f"{vid}: stale or mismatched registration session")
        if vid not in offsets:
            reasons.append(f"{vid}: no current registered offset")
        if not fresh(agent, 2.0):
            reasons.append(f"{vid}: agent status missing or stale")
        else:
            for name in ("registered", "pose_valid", "altitude_ok"):
                if agent.get(name) is not True:
                    reasons.append(f"{vid}: {name} is false")
            if agent.get("state") not in (
                {"MISSION"} if require_mission else {"HOLD", "MISSION"}
            ):
                reasons.append(f"{vid}: wrong MULLET state")
            if require_mission and (
                not agent.get("commands_enabled")
                or not agent.get("commands_sent", 0)
                or not isinstance(agent.get("last_command_time"), (float, int))
                or not 0 <= now - agent["last_command_time"] <= settings.pose_age
            ):
                reasons.append(f"{vid}: no fresh command stream")
        if not fresh(pose, settings.pose_age):
            reasons.append(f"{vid}: map pose missing or stale")
        if (
            not fresh(plan, settings.plan_expiry)
            or accepted_velocity(
                plan,
                scenario_hash,
                vid,
                now,
                registered=True,
                mission=True,
                max_speed=settings.max_speed_for_role(vehicle["role"]),
                position=pose.get("position") if isinstance(pose, dict) else None,
                bounds=settings.bounds,
                margin=settings.margin,
                horizon=settings.horizon,
            )
            is None
        ):
            reasons.append(f"{vid}: no bounded fresh velocity proposal")
        vehicles[vid] = {
            "role": vehicle["role"],
            "agent": agent,
            "pose": pose,
            "plan": plan,
            "offset": offsets.get(vid),
        }
    return {
        "ready": not reasons,
        "timestamp": now,
        "scenario_hash": scenario_hash,
        "require_mission": require_mission,
        "reasons": reasons,
        "central": status,
        "vehicles": vehicles,
    }
