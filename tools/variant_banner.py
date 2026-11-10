"""The line under the title of every duck's BOM and print README, naming which duck it is for, so nobody orders or prints for the wrong one
(2026-11-06, the user: "so I don't accidentally order parts for a different variant").

  python -I tools/variant_banner.py        stamps every BOM in docs/<duck>/ and every README in prints/<duck>/ (make_docs.py and export_prints.py do it as they write)
"""
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NAMES = {'dynamixel': 'all-Dynamixel duck', 'hd1910': 'Feetech HD-1910 duck', 'feetech': 'all-Feetech duck',
         'hybrid': 'hybrid duck (STS3215 legs, Dynamixel head)', 'hobby': 'hobby-servo duck', 'nano': 'nano duck', 'big': 'big duck'}
SHORT = {'dynamixel': 'all-Dynamixel', 'hd1910': 'HD-1910', 'feetech': 'all-Feetech', 'hybrid': 'hybrid', 'hobby': 'hobby-servo', 'nano': 'nano', 'big': 'big'}
MARK = '<!-- variant banner -->'


def banner(recipe, kind):
    """kind 'bom' (docs/<duck>/BOM.md) or 'prints' (prints/<duck>/README.md)."""
    if kind == 'bom':
        others = [f'[{SHORT[r]}](../{r}/BOM.md)' for r in NAMES if r != recipe and os.path.exists(os.path.join(ROOT, 'docs', r, 'BOM.md'))]
        what = 'This is the parts list'
    else:
        others = [f'[{SHORT[r]}](../{r}/README.md)' for r in NAMES if r != recipe and os.path.exists(os.path.join(ROOT, 'prints', r, 'README.md'))]
        what = 'These are the print files'
    return (f'> **{what} for the {NAMES[recipe]} only.** Each duck has its own parts, prints and BOM, and they don\'t mix. '
            f'Other ducks: {" · ".join(others)}. {MARK}')


def stamp(path, recipe, kind):
    """Put the banner under the file's first line (its title), or refresh it."""
    if not os.path.exists(path):
        return False
    lines = open(path, encoding='utf-8').read().split('\n')
    lines = [l for l in lines if MARK not in l]
    if len(lines) > 1 and lines[1] == '' and len(lines) > 2 and lines[2] == '':
        lines.pop(1)
    lines[1:1] = ['', banner(recipe, kind)]
    open(path, 'w', encoding='utf-8', newline='\n').write('\n'.join(lines))
    return True


def main():
    for r in NAMES:
        for kind, path in (('bom', os.path.join(ROOT, 'docs', r, 'BOM.md')), ('prints', os.path.join(ROOT, 'prints', r, 'README.md'))):
            if stamp(path, r, kind):
                print(os.path.relpath(path, ROOT))


if __name__ == '__main__':
    main()
