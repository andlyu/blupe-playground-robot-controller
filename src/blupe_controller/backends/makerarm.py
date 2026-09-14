"""MakerArm SDK process plus the shared single-arm operator and cloud bridge."""
def run(config):
    from ..makerarm import MakerArmDriver
    from ..operator import serve
    driver = MakerArmDriver(config).connect()
    try:
        return serve(driver, config)
    finally:
        driver.close()
