from setuptools import find_packages, setup

package_name = 'simulado_pf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='borg',
    maintainer_email='venanciofaf@al.insper.edu.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circuito = simulado_pf.circuito:main',
            'aleatorio = simulado_pf.aleatorio:main',
            'seguidores = simulado_pf.seguidores:main',
        ],
    },
)
