"""Instala el modelo de Fusion y sus recursos de simulación."""

from glob import glob

from setuptools import setup

package_name = 'bajana_solis_ws2_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + [
        ('share/' + package_name + '/' + directory, glob(directory + '/*'))
        for directory in ('launch', 'urdf', 'meshes', 'config', 'worlds')
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Grupo 14',
    maintainer_email='grupo14@example.invalid',
    description='Descripción y simulación del robot diferencial de Bajaña y Solis.',
    license='Apache-2.0',
)
