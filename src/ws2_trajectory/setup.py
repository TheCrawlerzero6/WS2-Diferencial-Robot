"""Instala el control del recorrido y sus pruebas de movimiento."""

from setuptools import find_packages, setup

package_name = 'ws2_trajectory'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    maintainer='Grupo 14',
    maintainer_email='grupo14@example.invalid',
    description='Recorrido del número 4 y pruebas de movimiento del grupo 14.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'trayectoria_4 = ws2_trajectory.trayectoria_4:main',
            'prueba_movimiento = ws2_trajectory.trayectoria_4:prueba_main',
        ],
    },
)
