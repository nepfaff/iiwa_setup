import os

from typing import List


def get_package_xmls() -> List[str]:
    """Returns a list of package.xml files."""
    path = os.path.abspath("models/package.xml")
    if os.path.exists(path):
        return [path]
    return []
