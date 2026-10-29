from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uc4_adc_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='koh',
    maintainer_email='khkoh23@gmail.com',
    description='TODO: Package description',
    license='PolyForm-Noncommercial-1.0.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'uc4_adc_driver = uc4_adc_driver.uc4_adc_driver:main',
        ],
    },
)
