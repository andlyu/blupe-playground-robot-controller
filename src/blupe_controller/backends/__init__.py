"""Lazy runtime registry: hardware imports occur only when run is invoked."""
from importlib import import_module

BACKENDS = {'yam': 'yam', 'so101': 'so101'}

def run(config):
    """Start the selected runtime without importing the other robot's SDK."""
    try:
        name = BACKENDS[config['hardware']]
    except (KeyError, TypeError):
        raise ValueError('Supported profiles: yam, so101') from None
    return import_module(f'{__name__}.{name}').run(config)
