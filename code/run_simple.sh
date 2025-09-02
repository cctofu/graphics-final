echo "Compiling document"
gcc-13 -fopenmp -o main main_simple.cpp -lstdc++
time ./main 500