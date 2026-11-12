#!/usr/bin/env python3
"""Prepare research-only SuperMimic assets from one exported K1 run."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
import yaml


def reference_features(data, defaults):
    names = list(map(str, data['body_names']))
    root = names.index('pelvis')
    q = np.asarray(data['body_quat_w'][:, root, :], dtype=np.float64)
    if q.ndim != 2 or q.shape[1] != 4 or np.any(np.abs(np.linalg.norm(q, axis=1) - 1) > 1e-4):
        raise ValueError('invalid root quaternions')

    def inverse_rotate(quat, vec):
        xyz = -quat[:, 1:]
        w = quat[:, 0, None]
        t = 2 * np.cross(xyz, vec)
        return vec + w * t + np.cross(xyz, t)

    gravity = np.tile([0., 0., -1.], (len(q), 1))
    w, x, y, z = q.T
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    heading = np.stack(
        [np.cos(yaw / 2), np.zeros_like(yaw), np.zeros_like(yaw), np.sin(yaw / 2)], axis=1)
    out = np.concatenate([
        data['joint_pos'] - defaults, data['joint_vel'], inverse_rotate(q, gravity),
        inverse_rotate(q, data['body_ang_vel_w'][:, root, :]),
        inverse_rotate(heading, data['body_lin_vel_w'][:, root, :]),
        data['body_pos_w'][:, root, 2, None]], axis=1)
    if out.shape[1] != 56 or not np.isfinite(out).all():
        raise ValueError('invalid reference features')
    return out


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(source, dest):
    source, dest = Path(source), Path(dest)
    if dest.exists():
        raise FileExistsError(f'refusing to overwrite {dest}')
    cfg = yaml.safe_load((source / 'params/sim2real.yaml').read_text())
    names = cfg['policy_joints']
    if len(names) != 23 or cfg['step_dt'] != .02:
        raise ValueError('expected 23 joints and 50 Hz')
    old = cfg['observations']
    if (list(old) != ['supermimic_policy_history', 'supermimic_motion_goal'] or
            len(old['supermimic_motion_goal']['scale']) != 566):
        raise ValueError('expected orientation-only SuperMimic export')
    defaults = np.array([cfg['joint_properties'][n]['default_position'] for n in names])

    def term(n, h):
        return {'params': {}, 'clip': None, 'scale': [1.0] * n, 'history_length': h}

    cfg['observations'] = {
        'joint_pos_rel': term(23, 10), 'joint_vel_rel': term(23, 10),
        'base_ang_vel': term(3, 10), 'projected_gravity': term(3, 10),
        'last_action': term(23, 10), 'supermimic_future': term(560, 1),
        'motion_anchor_ori_b': term(6, 1)}
    dest.mkdir(parents=True)
    (dest / 'params').mkdir()
    (dest / 'exported').mkdir()
    shutil.copy2(source / 'exported/policy.onnx', dest / 'exported/policy.onnx')
    (dest / 'params/sim2real.yaml').write_text(
        '# Research only; validate parity and Sim2Sim before hardware.\n' +
        yaml.safe_dump(cfg, sort_keys=False))
    manifest = {
        'source': str(source), 'source_onnx_sha256': sha(source / 'exported/policy.onnx'),
        'motion': {}}
    for i in range(1, 8):
        name = f'taeguek_{i}'
        npz = source / f'params/{name}.npz'
        csv = source / f'params/{name}.csv'
        with np.load(npz, allow_pickle=False) as data:
            if (list(map(str, data['joint_names'])) != names or
                    float(data['fps'].reshape(-1)[0]) != 50.):
                raise ValueError(f'{name}: joint order or fps mismatch')
            features = reference_features(data, defaults)
        shutil.copy2(csv, dest / 'params' / csv.name)
        side = dest / 'params' / f'{name}.csv.supermimic'
        np.savetxt(side, features, delimiter=',', fmt='%.9g')
        manifest['motion'][name] = {
            'npz_sha256': sha(npz), 'csv_sha256': sha(csv),
            'sidecar_sha256': sha(side), 'frames': len(features)}
    manifest['policy_onnx_sha256'] = sha(dest / 'exported/policy.onnx')
    manifest['config_sha256'] = sha(dest / 'params/sim2real.yaml')
    (dest / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('source', type=Path)
    ap.add_argument('destination', type=Path)
    args = ap.parse_args()
    prepare(args.source, args.destination)
