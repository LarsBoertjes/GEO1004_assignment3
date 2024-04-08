# GEO1004_assignment3

## Installation Instructions

Before running the C++ code, ensure that the following libraries are installed in your C++ environment:

- CGAL
- nlohmann JSON for C++ package (although it's also included in the repository)

## Usage Instructions

1. **Specify .OBJ File**: 
    - In the `main.cpp` file, navigate to line 36 and specify the filepath to an .OBJ-file.
    - We recommend using the files provided in the `data` folder, as some parts of the code are hardcoded for these files. It is set to "Wellness_center.obj" by default.

2. **Specify Resolution**: 
    - On line 68 of `main.cpp`, specify the desired resolution.

3. **Run the Code**: 
    - After setting the filepath and resolution, compile and run `main.cpp`.
    - The code will output a CityJSON file to your target folder.

## Additional Notes

- Ensure that all necessary dependencies are properly configured and accessible in your development environment.
- Running of a resolution lower than 0.2 will take some time
