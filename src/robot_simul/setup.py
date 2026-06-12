from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'agv_robot'

data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
]

# models 폴더 안의 각 하위 폴더와 파일을 재귀적으로 추가
def package_files(directory):
    paths = []
    for (path, _, filenames) in os.walk(directory):
        if not filenames:
            continue
        # 'models/Speed limit sign/meshes/' → 'share/agv_robot/models/Speed limit sign/meshes/'
        install_dir = os.path.join('share', package_name, path)
        files = [os.path.join(path, f) for f in filenames]
        paths.append((install_dir, files))
    return paths

data_files += package_files('models')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jsdbswjd',
    maintainer_email='jsdbswjd@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
