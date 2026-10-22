# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0.
"""Read-only capability checks before launching hardware, never opens a DXL port.

Binary markers catch older installations that silently ignore the capture option.
They do not certify ABI compatibility or real-time performance.
"""
from pathlib import Path
import re
import subprocess


def check_response_evidence(dhi_library):
    """Check the selected DHI and its resolved SDK dependency without loading DHI."""
    dhi = Path(dhi_library).resolve(strict=True)
    if b'diagnostics_response_evidence_enabled' not in dhi.read_bytes():
        raise RuntimeError(f'Response evidence is unsupported by selected DHI: {dhi}')
    dependencies = subprocess.run(
        ['ldd', str(dhi)], text=True, capture_output=True, timeout=10, check=True).stdout
    if 'not found' in dependencies:
        raise RuntimeError(f'DHI has unresolved shared-library dependencies: {dhi}')
    match = re.search(r'^\s*libdynamixel_sdk\.so\s+=>\s+(\S+)\s+\(',
                      dependencies, flags=re.MULTILINE)
    if not match:
        raise RuntimeError(f'Cannot identify the SDK resolved for DHI: {dhi}')
    sdk = Path(match[1]).resolve(strict=True)
    if b'ScopedRxFailureCapture' not in sdk.read_bytes():
        raise RuntimeError(f'Response evidence is unsupported by resolved SDK: {sdk}')
    return {'dhi_library': str(dhi), 'sdk_library': str(sdk),
            'scope': 'capability markers only; ABI and 1 kHz performance not certified'}


def validate_response_evidence_launch(context):
    """OpaqueFunction callback; OFF leaves existing bringup behavior unchanged."""
    def enabled(name):
        value = context.launch_configurations.get(name, 'false').lower()
        if value not in ('true', 'false', '1', '0'):
            raise RuntimeError(f'{name} must be true/false or 1/0')
        return value in ('true', '1')

    if not enabled('diagnostics_response_evidence_enabled'):
        return []
    if not enabled('diagnostics_enabled'):
        raise RuntimeError('Response evidence requires diagnostics_enabled:=true')
    if enabled('use_mock_hardware'):
        return []  # Mock description does not instantiate the DHI plugin.
    from ament_index_python.packages import get_package_prefix
    prefix = Path(get_package_prefix('dynamixel_hardware_interface'))
    library = prefix / 'lib/libdynamixel_hardware_interface.so'
    check_response_evidence(library)
    return []
