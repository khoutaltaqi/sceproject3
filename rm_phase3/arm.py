"""Set and verify the Phase 2 arm coordinates."""
import argparse
from .common import EP, folder, save, settings


def main():
    argparse.ArgumentParser(description=__doc__).parse_args()
    c, out = settings(), folder('arm')
    with EP(c, video=False) as ep:
        actual = ep.arm()
        save(out/'arm.json', {'requested_mm': [c['arm_x_mm'], c['arm_y_mm']],
                             'measured_mm': actual})
        print(f'Arm x={actual[0]} mm, y={actual[1]} mm. Saved: {out}')


if __name__ == '__main__':
    main()
