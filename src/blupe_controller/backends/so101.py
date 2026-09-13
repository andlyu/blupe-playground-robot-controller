"""SO101 runtime: LeRobot adapter plus local operator and cloud bridge."""
def run(config):
    from ..so101 import SO101Driver
    from ..operator import serve
    driver = SO101Driver(config).connect()
    try:
        return serve(driver, config)
    finally:
        driver.close()
