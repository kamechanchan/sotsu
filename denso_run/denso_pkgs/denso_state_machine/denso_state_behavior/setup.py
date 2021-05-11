import os
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

__pkgname__ = 'denso_state_behavior'


def package_datas(dir_list):
    prefix_path = os.path.join('src', __pkgname__)
    data_files = []

    for directory in dir_list:
        # Recursive-Search files
        start_point = os.path.join(prefix_path, directory)
        for root, dirs, files in os.walk(start_point):
            for file_name in files:
                # Append file path that is removed prefix path (e.g.
                # 'src/pkgname/aaa.txt' -> 'aaa.txt')
                data_files.append(
                    os.path.join(
                        root, file_name)[
                        len(prefix_path) + 1:])

    return data_files


setup_args = generate_distutils_setup(
    packages=[__pkgname__],
    package_dir={'': 'src'},
    package_data={__pkgname__: package_datas(['data'])}
)

setup(**setup_args)
