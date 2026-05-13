from setuptools import setup

package_name = 'workspace_fk_visualizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    author='Ali',
    maintainer='Ali',
    maintainer_email='ali@todo.todo',
    description='FK workspace publisher for RA6A arm without MoveIt2 Python',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'fk_workspace_py = workspace_fk_visualizer.fk_workspace_py:main',
        ],
    },
)
