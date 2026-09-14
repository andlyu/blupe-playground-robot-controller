"""Bimanual calibrated controller, with read-only bring-up fallback."""
def run(config):
    from ..bimanual_so101 import BimanualSO101Driver, BimanualSO101Monitor
    from ..operator import serve
    cls = BimanualSO101Driver if config['settings'].get('calibrations') else BimanualSO101Monitor
    driver = cls(config).connect()
    try:
        return serve(driver, config)
    finally:
        driver.close()
