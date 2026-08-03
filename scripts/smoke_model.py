from pathlib import Path

import mujoco


def main() -> None:
    model_path = Path(__file__).resolve().parents[1] / "assets" / "so101" / "so101_new_calib.xml"
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
    print(f"model_path={model_path}")
    print(f"nq={model.nq} nv={model.nv} nu={model.nu} nsite={model.nsite}")
    print(f"gripperframe_site_id={site_id}")
    print(f"gripperframe_xyz={data.site_xpos[site_id].round(5).tolist()}")

    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        print(f"joint[{i}] {name} range={model.jnt_range[i].round(5).tolist()}")


if __name__ == "__main__":
    main()
