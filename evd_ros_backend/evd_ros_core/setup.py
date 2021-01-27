from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=[
        'evd_script',
        'evd_interfaces',
        'evd_version_tracking'
    ],
    scripts=[''],
    package_dir={'':'src'})

setup(**setup_args)
