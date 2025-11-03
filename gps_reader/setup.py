from setuptools import setup
import os

package_name = 'gps_reader'

setup(
 name=package_name,
 version='0.0.1',
 packages=[package_name],
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml'])
   ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='Andrejs Z.',
 maintainer_email='andrejs.zujevs@lmt.lv',
 description='GC GUI in Kivy',
 license='LMT',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'gps_reader_node = gps_reader.gps_reader:main'
     ],
   },
)