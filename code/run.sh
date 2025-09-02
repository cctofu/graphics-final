echo "Compiling document..."
gcc-13 -fopenmp -o main main.cpp -lstdc++

echo "Creating scene 1..."
time ./main testcases/scene1.txt scene1.png 5000

echo "Creating scene 2..."
time ./main testcases/scene2.txt scene2.png 5000