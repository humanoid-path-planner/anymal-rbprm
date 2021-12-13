from pathlib import Path

def prefix():
    """$prefix/lib/pythonX.Y/site-packages/$module/__init__.py: extract prefix from module"""
    return Path(__file__).parent.parent.parent.parent.parent
