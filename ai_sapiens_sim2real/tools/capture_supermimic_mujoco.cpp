#include "robotis_mujoco/reference_motion.h"
#include "robotis_mujoco/policy_observation_builder.h"
#include "robotis_mujoco/policy_runner.h"
#include "robotis_mujoco/native_pd_control.h"
#include <mujoco/mujoco.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

static void check(bool b, const std::string & e) {if(!b) {throw std::runtime_error(e);}}
static double yaw(const mjtNum * q)
{
  return std::atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
}
int main(int argc, char **argv)
{
  if(argc != 8 && argc != 9) {
    std::cerr <<
      "usage: model policy_dir reference_npz policy_ref_csv_or_npz fx fy output.csv [full]\n";
    return 2;
  }
  try {
    char err[1024]{};
    std::unique_ptr<mjModel,
      decltype(& mj_deleteModel)> m(mj_loadXML(argv[1], nullptr, err, sizeof(err)), mj_deleteModel);
    check(bool(m), err);
    m->opt.timestep = .005;native_pd::configure(m.get());
    std::unique_ptr<mjData, decltype(& mj_deleteData)> d(mj_makeData(m.get()), mj_deleteData),
    ghost(mj_makeData(m.get()), mj_deleteData);check(bool(d) && bool(ghost), "mj_makeData");
    std::string p = argv[2], e;PolicyConfig cfg;ReferenceMotion metric, reference;
    PolicyObservationBuilder builder;PolicyRunner runner;
    check(cfg.load_config(p + "/params/sim2real.yaml", &e), e);
    check(metric.load_npz(argv[3], &e), e);
    std::string ref = argv[4];bool csv = ref.size() >= 4 && ref.substr(ref.size() - 4) == ".csv";
    check(csv ? reference.load_csv(ref, 50, cfg.policy_joints(), &e) : reference.load_npz(ref, &e),
      e);
    check(reference.frame_count() == metric.frame_count() && reference.fps() == 50 &&
      cfg.step_dt() == .02, "reference/timing mismatch");
    check(builder.configure(m.get(), cfg, &reference, &e), e);
    check(runner.load_policy(p + "/exported/policy.onnx", &e), e);
    auto names = cfg.policy_joints();check(names.size() == 23,
  "expected 23 joints");std::vector<int> joints, acts;
    for(auto & n:names) {
      int j = mj_name2id(m.get(), mjOBJ_JOINT, n.c_str()), a = -1;for(int k = 0; k < m->nu; ++k) {
        if(m->actuator_trnid[2 * k] == j) {
          a = k;
        }
      }
      check(j >= 0 && a >= 0, "missing actuator " + n);joints.push_back(j);acts.push_back(a);
    }
    std::vector<float> pos, quat, lin, ang, jpos, jvel;metric.get_initial_base_pose(pos, quat);
    metric.get_initial_base_velocity(lin, ang);metric.get_initial_joint_position(jpos);
    metric.get_initial_joint_velocity(jvel);
    check(pos.size() == 3 && quat.size() == 4 && jpos.size() == 23, "bad initial state");
    for(int i = 0; i < 3; ++i) {
      d->qpos[i] = pos[i];d->qvel[i] = lin[i];
    }
    for(int i = 0; i < 4; ++i) {
      d->qpos[i + 3] = quat[i];
    }
    mjtNum inv[4], av[3] = {ang[0], ang[1], ang[2]};mju_negQuat(inv, d->qpos + 3);
    mju_rotVecQuat(d->qvel + 3, av, inv);
    for(int k = 0; k < 23; ++k) {
      d->qpos[m->jnt_qposadr[joints[k]]] = jpos[k];d->qvel[m->jnt_dofadr[joints[k]]] = jvel[k];
      native_pd::set_target(m.get(), d.get(), acts[k], jpos[k], cfg.joint_stiffness()[k],
        cfg.joint_damping()[k]);
    }
    mj_forward(m.get(), d.get());builder.reset_history();
    int pelvis = mj_name2id(m.get(), mjOBJ_BODY, "pelvis"),
      left = mj_name2id(m.get(), mjOBJ_BODY, "left_ankle_roll_link"),
      right = mj_name2id(m.get(), mjOBJ_BODY, "right_ankle_roll_link");
    check(pelvis > 0 && left > 0 && right > 0, "body not found");
    std::ofstream cap(std::string(argv[7]) + ".capture");cap.precision(17);
    std::ofstream f(argv[7]);check(bool(f), "output open failed");
    f <<
      "time,xy_error_m,yaw_error_deg,joint_rmse_rad,ankle_mean_error_m,root_height,up_z,reference_height,reference_up_z\n";
    double fx = std::stod(argv[5]), fy = std::stod(argv[6]);
    std::vector<float> obs, act(23, 0), jp, rp, rq;
    const int steps = argc == 9 && std::string(argv[8]) == "full" ? metric.frame_count() * 4 : 4000;
    for(int step = 0; step < steps; ++step) {
      double t = step * .005;
      if(step % 4 == 0) {
        metric.get_base_pose(t, rp, rq);metric.get_joint_position(t, jp);
        check(rp.size() == 3 && rq.size() == 4 && jp.size() == 23, "reference missing");
        for(int i = 0; i < 3; ++i) {
          ghost->qpos[i] = rp[i];
        }
        for(int i = 0; i < 4; ++i) {
          ghost->qpos[i + 3] = rq[i];
        }
        for(int k = 0; k < 23; ++k) {
          ghost->qpos[m->jnt_qposadr[joints[k]]] = jp[k];
        }
        mj_forward(m.get(), ghost.get());
        double xy = std::hypot(d->qpos[0] - rp[0], d->qpos[1] - rp[1]);
        mjtNum rqnum[4] = {rq[0], rq[1], rq[2], rq[3]};
        double ye = std::remainder(yaw(d->qpos + 3) - yaw(rqnum), 2 * M_PI) * 180 / M_PI;
        double sq = 0;for(int k = 0; k < 23; ++k) {
          double diff = d->qpos[m->jnt_qposadr[joints[k]]] - jp[k];sq += diff * diff;
        }
        double ankle = 0;for(int b:{left, right}) {
          double s = 0;for(int k = 0; k < 3; ++k) {
            double diff = d->xpos[3 * b + k] - ghost->xpos[3 * b + k];s += diff * diff;
          }
          ankle += std::sqrt(s) / 2;
        }
        double up = d->xmat[9 * pelvis + 8], ref_up = 1 - 2 * (rq[1] * rq[1] + rq[2] * rq[2]);
        f << t << ',' << xy << ',' << std::abs(ye) << ',' << std::sqrt(sq / 23) << ',' << ankle <<
          ',' << d->qpos[2] << ',' << up << ',' << rp[2] << ',' << ref_up << '\n';
        if(!std::isfinite(xy) || std::abs(d->qpos[2] - rp[2]) > .25 || std::abs(up - ref_up) > .8) {
          break;
        }
        check(builder.build(d.get(), t, act, obs, &e), e);
        cap << t; for(int k = 0; k < 4; ++k) {
          cap << ',' << d->xquat[4 * pelvis + k];
        }
        mjtNum rootvel[6];mj_objectVelocity(m.get(), d.get(), mjOBJ_XBODY, pelvis, rootvel, 1);
        for(int k = 0;
          k < 3; ++k)
        {
          cap << ',' << rootvel[k];
        }
        for(int k = 0; k < 23; ++k) {
          cap << ',' << d->qpos[m->jnt_qposadr[joints[k]]];
        }
        for(int k = 0; k < 23; ++k) {
          cap << ',' << d->qvel[m->jnt_dofadr[joints[k]]];
        }
        for(float a:act) {
          cap << ',' << a;
        }
        for(float v:obs) {
          cap << ',' << v;
        }
        cap << '\n';
        check(runner.run_inference(obs, act, &e), e);cfg.clip_actions(act);
        check(act.size() == 23 && std::all_of(act.begin(), act.end(), [](float x){
            return std::isfinite(x);
                                                                                                          }),
          "bad action");
        for(int k = 0; k < 23; ++k) {
          native_pd::set_target(m.get(), d.get(), acts[k],
            cfg.actions()[0].position_target(k, act[k]), cfg.joint_stiffness()[k],
            cfg.joint_damping()[k]);
        }
      }
      d->xfrc_applied[6 * pelvis] = (t >= 8 && t < 8.2) ? fx : 0;
      d->xfrc_applied[6 * pelvis + 1] = (t >= 8 && t < 8.2) ? fy : 0;
      mj_step(m.get(), d.get());mj_forward(m.get(), d.get());
    }
  } catch(const std::exception & e) {
    std::cerr << e.what() << '\n';return 1;
  }
  return 0;
}
