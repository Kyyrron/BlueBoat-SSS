"""Display start-up ordering (project_review.md §2, gap G4).

``RightPanel`` emitted its first ``display_changed`` at construction —
before the window had connected anything — so the two services started
on ``DisplaySettings()`` (γ 1.0) while the slider showed ``display.gamma``
(0.7) and the AI pictures used the config value: three transfers at
once, and ``metadata.json``'s ``display_settings_at_end`` was null. Now
the services are seeded from the config and the window re-emits the
panel after wiring it.
"""

from __future__ import annotations

import pytest

from blueboat_gcs.mapping.renderer import DisplaySettings
from test_recording_guards import FakeAcquisition


def test_config_gamma_differs_from_the_dataclass_default(tmp_config):
    """The bug only exists because these two differ; if they ever agree
    the ordering guard below is still the contract."""
    assert tmp_config.display.gamma != DisplaySettings().gamma


def test_services_and_metadata_start_on_the_slider_value(qapp, tmp_config,
                                                         no_modal_dialogs):
    from blueboat_gcs.core.mosaic_service import MosaicService
    from blueboat_gcs.core.signals import AppSignals
    from blueboat_gcs.gui.main_window import MainWindow

    signals = AppSignals()
    mosaic = MosaicService(tmp_config)
    # Seeded before the panel exists.
    assert mosaic._renderer.settings.gamma == pytest.approx(tmp_config.display.gamma)
    window = MainWindow(tmp_config, signals, mosaic, FakeAcquisition())
    try:
        g = tmp_config.display.gamma
        assert window.waterfall_service._renderer.settings.gamma == pytest.approx(g)
        assert mosaic._renderer.settings.gamma == pytest.approx(g)
        # The re-emission reached the recording manager: metadata.json
        # will carry the settings instead of null.
        assert window.recording._display_settings is not None
        assert window.recording._display_settings.gamma == pytest.approx(g)
        # After the debounce timers fire nothing has drifted.
        from PySide6.QtTest import QTest
        QTest.qWait(200)
        assert window.waterfall_service._renderer.settings.gamma == pytest.approx(g)
        assert mosaic._renderer.settings.gamma == pytest.approx(g)
        assert window.right_panel._gamma.value() == int(round(g * 100))
    finally:
        window.recording._pending_adoption = None
        window.close()
        qapp.processEvents()


def test_the_contrast_slider_reaches_the_saved_pictures(qapp, tmp_config,
                                                        no_modal_dialogs):
    """One transfer everywhere, including the PNGs (2026-09-18, G13).

    The slider drove the screen and the mosaic but not ``DisplayModel.gamma``,
    which is what ``SeabedImager.to_png8`` writes through: a session run at
    the operator's contrast saved its AI pictures at the config's instead
    (0.48 on screen against 0.7 on disk in the 2026-09-07 session). Each
    picture stores the gamma it was written with, so this is checked where
    it is decided -- on the model the imager holds.
    """
    from blueboat_gcs.core.mosaic_service import MosaicService
    from blueboat_gcs.core.signals import AppSignals
    from blueboat_gcs.gui.main_window import MainWindow

    signals = AppSignals()
    mosaic = MosaicService(tmp_config)
    window = MainWindow(tmp_config, signals, mosaic, FakeAcquisition())
    try:
        g0 = tmp_config.display.gamma
        assert window.display_model.gamma == pytest.approx(g0)
        # The imager writes through the same model object.
        assert window.seabed_imager._model is window.display_model

        window._on_display_changed(DisplaySettings(gamma=0.48))
        assert window.display_model.gamma == pytest.approx(0.48)
        assert window.seabed_imager._model.gamma == pytest.approx(0.48)
        # ... and the snapshot the pictures are rendered with follows it,
        # rather than being cached at the old value.
        assert window.display_model.snapshot().gamma == pytest.approx(0.48)

        # A nonsense value is ignored rather than blanking every picture.
        window._on_display_changed(DisplaySettings(gamma=0.0))
        assert window.display_model.gamma == pytest.approx(0.48)
    finally:
        window.recording._pending_adoption = None
        window.close()
        qapp.processEvents()
