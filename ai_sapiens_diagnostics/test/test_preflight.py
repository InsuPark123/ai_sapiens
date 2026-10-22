"""Catch silently ignored capture options before hardware processes start."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from ai_sapiens_diagnostics import preflight


def context(**kwargs):
    return SimpleNamespace(launch_configurations=kwargs)


def test_off_never_inspects_driver(monkeypatch):
    probe = Mock(side_effect=AssertionError('OFF must not inspect drivers'))
    monkeypatch.setattr(preflight, 'check_response_evidence', probe)
    assert preflight.validate_response_evidence_launch(context()) == []
    probe.assert_not_called()


def test_capture_without_diagnostics_rejected_before_hardware():
    with pytest.raises(RuntimeError, match='requires diagnostics_enabled'):
        preflight.validate_response_evidence_launch(context(
            diagnostics_response_evidence_enabled='true', diagnostics_enabled='false'))


def test_mock_does_not_require_real_driver(monkeypatch):
    probe = Mock(side_effect=AssertionError('Mock must not inspect real drivers'))
    monkeypatch.setattr(preflight, 'check_response_evidence', probe)
    assert preflight.validate_response_evidence_launch(context(
        diagnostics_response_evidence_enabled='true', diagnostics_enabled='true',
        use_mock_hardware='true')) == []
    probe.assert_not_called()


def test_invalid_capture_flag_rejected():
    with pytest.raises(RuntimeError, match='true/false'):
        preflight.validate_response_evidence_launch(context(
            diagnostics_response_evidence_enabled='maybe'))


def test_old_dhi_rejected_before_sdk_resolution(tmp_path, monkeypatch):
    driver = tmp_path / 'driver.so'
    driver.write_bytes(b'older driver')
    probe = Mock(side_effect=AssertionError('Must reject old DHI first'))
    monkeypatch.setattr(preflight.subprocess, 'run', probe)
    with pytest.raises(RuntimeError, match='unsupported by selected DHI'):
        preflight.check_response_evidence(driver)
    probe.assert_not_called()


@pytest.mark.parametrize('dependencies', [
    '\tlibdynamixel_sdk.so => not found\n',
    '\tlibdynamixel_sdk.so => /some/sdk.so (0x123)\n\tlibhardware_interface.so => not found\n',
    '\tlibother.so => /lib/other.so (0x123)\n',
])
def test_unresolved_or_unidentified_sdk_rejected(tmp_path, monkeypatch, dependencies):
    driver = tmp_path / 'driver.so'
    driver.write_bytes(b'diagnostics_response_evidence_enabled')
    monkeypatch.setattr(preflight.subprocess, 'run',
                        Mock(return_value=SimpleNamespace(stdout=dependencies)))
    with pytest.raises(RuntimeError):
        preflight.check_response_evidence(driver)


@pytest.mark.parametrize('supported', [False, True])
def test_inspects_resolved_sdk_not_a_different_install(tmp_path, monkeypatch, supported):
    driver = tmp_path / 'driver.so'
    driver.write_bytes(b'diagnostics_response_evidence_enabled')
    resolved_sdk = tmp_path / 'selected_sdk.so'
    resolved_sdk.write_bytes(b'ScopedRxFailureCapture' if supported else b'older SDK')
    # A compatible SDK existing elsewhere does not make the selected SDK compatible.
    (tmp_path / 'unused_sdk.so').write_bytes(b'ScopedRxFailureCapture')
    dependencies = f'\tlibdynamixel_sdk.so => {resolved_sdk} (0x123)\n'
    monkeypatch.setattr(preflight.subprocess, 'run',
                        Mock(return_value=SimpleNamespace(stdout=dependencies)))
    if supported:
        result = preflight.check_response_evidence(driver)
        assert Path(result['sdk_library']) == resolved_sdk
        assert 'not certified' in result['scope']
    else:
        with pytest.raises(RuntimeError, match='unsupported by resolved SDK'):
            preflight.check_response_evidence(driver)
