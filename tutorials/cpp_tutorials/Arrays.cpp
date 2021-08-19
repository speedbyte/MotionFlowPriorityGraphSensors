//
// Created by veikas on 23.07.17.
//

#include "Arrays.h"
#include <iostream>
#include <array>
#include <vector>
#include <iterator>
#include <algorithm>

typedef struct {
    int a;
    int b;
} STRUCT_AGGREGRATE;

int main ( int argc, char *argv[]) {

    std::srand(time(0));

    STRUCT_AGGREGRATE struct_aggregrate{2,3};
    STRUCT_AGGREGRATE array_struct_aggregrate[2] = {{2,3},{2,3}};
    std::array<STRUCT_AGGREGRATE,2> std_array_struct_aggregrate = {{{2,3},{2,3}}};

    std::vector<ushort> m_xPos(10,100);


    std::cout << struct_aggregrate.a << std::endl;
    std::cout << "----------1D access---------------------------------------\n" ;
    /**
     * xy_pts is the pointer to the first integer in the array. The content of xy_pts is the address of the first
     * integer in the array. The content of the address can be accesses by the subscript operator.
     * If xy_pts is incremented, then it jumps an integer. Again the content of the address
     * can be accessed by the [] operator. (xy_pts+i)[0] or *(xy_pts + i)
     */
    int xy_pts[10] = {1,2,3,4,5,6,7,8,9,10};
    for ( unsigned i = 0; i < 10; i++) {
        std::cout << xy_pts[i] << std::endl;
        std::cout << (xy_pts+i)[0] << std::endl;
        std::cout << *(xy_pts + i) << std::endl;
    }

    std::cout << "-----------1D ptr access--------------------------------------\n" ;
    /**
     * xy_pts_ptr is the pointer to an array of 10 integers. The content of xy_pts_ptr is the address of the first
     * integer in the array of 10 integers. The content of the address can be accessed by the subsrippt operator. If
     * xy_pts_ptr is incremented, then it jumps the array of 10 integers.
     */
    int (*xy_pts_ptr)[10] = &xy_pts;   // Pointer to an array of 10 integers
    for ( unsigned i = 0; i < 10; i++) {
        std::cout << xy_pts_ptr[i] << std::endl;  // jump by array address
        std::cout << *(xy_pts_ptr+i) << std::endl; // jump by array address
        std::cout << *(*xy_pts_ptr+i) << std::endl; // correct access
        std::cout << (*xy_pts_ptr)[i] << std::endl; // correct access
        std::cout << (*xy_pts_ptr+i)[0] << std::endl; // correct access
        std::cout << *(*(xy_pts_ptr + i)) << std::endl; // jump by array access. Incorrect

    }

    std::cout << "------------2D  access -------------------------------------\n";
    int xy_pts_2d[4][3] = {{1,2,3},{4,5,6},{7,8,9},{10,11,12}}; // xy_pts is the pointer to the first element in the array.
    int x_dim = sizeof(xy_pts_2d)/sizeof(*xy_pts_2d);
    int y_dim = sizeof(xy_pts_2d)/sizeof(**xy_pts_2d)/x_dim;
    for ( unsigned i = 0; i < x_dim; i++) {
        for ( unsigned j = 0; j < y_dim; j++) {
            std::cout << xy_pts_2d[i][j] << std::endl;
            std::cout << (*(xy_pts_2d+i))[j] << std::endl;
            std::cout << *(*(xy_pts_2d+i)+j) << std::endl;
        }
    }

    std::cout << "------------2D Ptr access-------------------------------------\n";
    int (*xy_pts_2d_ptr)[4][3] = &xy_pts_2d;
    for ( unsigned i = 0; i < x_dim; i++) {
        for ( unsigned j = 0; j < y_dim; j++) {
            std::cout << *(*(*xy_pts_2d_ptr+i)+j) << std::endl;
            std::cout << (*(*xy_pts_2d_ptr+i)+j)[0] << std::endl;
            std::cout << (*(*xy_pts_2d_ptr+i))[j] << std::endl;
        }
    }


    std::cout << "-----------1D std::array--------------------------------------\n" ;
    std::array<int,10> xy_pts_array = {1,2,3,4,5,6,7,8,9,10};
    for ( unsigned i = 0; i < xy_pts_array.size(); i++) {
        std::cout << xy_pts_array[i] << std::endl;
        std::cout << xy_pts_array.at(i) << std::endl;
        std::cout << xy_pts_array.operator[](i) << std::endl;
    }

    std::cout << "-----------1D std::array ptr--------------------------------------\n" ;
    std::array<int,10> *xy_pts_array_ptr = &xy_pts_array;
    for ( unsigned i = 0; i < xy_pts_array_ptr->size(); i++) {
        std::cout << xy_pts_array_ptr->at(i) << std::endl;
        std::cout << xy_pts_array_ptr->operator[](i) << std::endl;
        std::cout << (*xy_pts_array_ptr)[i] << std::endl;
    }

    std::cout << "-------------2D std::array------------------------------------\n";
    std::array< std::array<int,3>,4> xy_pts_array_2d = {{{1,2,3},{4,5,6},{7,8,9},{10,11,12}} };
    for ( unsigned i = 0; i < xy_pts_array_2d.size(); i++) {
        for (unsigned j = 0; j < xy_pts_array_2d.at(0).size(); j++) {
            std::cout << xy_pts_array_2d[i][j] << std::endl;
            std::cout << xy_pts_array_2d.at(i).at(j) << std::endl;
            std::cout << xy_pts_array_2d.operator[](i).operator[](j)<< std::endl;
        }
    }

    std::cout << "-------------2D std::array ptr------------------------------------\n";
    std::array< std::array<int,3>,4> *xy_pts_array_2d_ptr = &xy_pts_array_2d;
    for ( unsigned i = 0; i < xy_pts_array_2d.size(); i++) {
        for (unsigned j = 0; j < xy_pts_array_2d_ptr->at(0).size(); j++) {
            std::cout << *(((xy_pts_array_2d_ptr->data())+i)->data()+j) << std::endl;
            std::cout << xy_pts_array_2d_ptr->at(i).at(j) << std::endl;
            std::cout << xy_pts_array_2d_ptr->operator[](i).operator[](j)<< std::endl;
        }
    }


    std::cout << "-----------1D std::vector--------------------------------------\n" ;
    std::vector<int> xy_pts_vector(10);
    for ( unsigned i = 0; i < xy_pts_vector.size(); i++) {
        xy_pts_vector[i] = std::rand()%100;
    }
    for ( unsigned i = 0; i < xy_pts_vector.size(); i++) {
        std::cout << xy_pts_vector[i] << std::endl;
        std::cout << xy_pts_vector.at(i) << std::endl;
        std::cout << xy_pts_vector.operator[](i) << std::endl;
    }
    //for(const auto& array:m0) {
    std::copy(xy_pts_vector.begin(), xy_pts_vector.end(), std::ostream_iterator<int>(std::cout, " "));

    std::cout << "-----------1D std::vector ptr--------------------------------------\n" ;
    std::vector<int> *xy_pts_vector_ptr = &xy_pts_vector;
    for ( unsigned i = 0; i < xy_pts_vector_ptr->size(); i++) {
        std::cout << xy_pts_vector_ptr->at(i) << std::endl;
        std::cout << xy_pts_vector_ptr->operator[](i) << std::endl;
        std::cout << (*xy_pts_vector_ptr)[i] << std::endl;
    }

    std::cout << "-------------2D std::vector------------------------------------\n";
    std::vector< std::vector<int>> xy_pts_vector_2d = {{1,2,3},{4,5,6},{7,8,9},{10,11,12}};
    for ( unsigned i = 0; i < xy_pts_vector_2d.size(); i++) {
        for (unsigned j = 0; j < xy_pts_vector_2d.at(0).size(); j++) {
            std::cout << xy_pts_vector_2d[i][j] << std::endl;
            std::cout << xy_pts_vector_2d.at(i).at(j) << std::endl;
            std::cout << xy_pts_vector_2d.operator[](i).operator[](j)<< std::endl;
        }
    }

    std::cout << "-------------2D std::vector ptr------------------------------------\n";
    std::vector< std::vector<int>> *xy_pts_vector_2d_ptr = &xy_pts_vector_2d;
    for ( unsigned i = 0; i < xy_pts_vector_2d.size(); i++) {
        for (unsigned j = 0; j < xy_pts_vector_2d_ptr->at(0).size(); j++) {
            std::cout << *(((xy_pts_vector_2d_ptr->data())+i)->data()+j) << std::endl;
            std::cout << xy_pts_vector_2d_ptr->at(i).at(j) << std::endl;
            std::cout << xy_pts_vector_2d_ptr->operator[](i).operator[](j)<< std::endl;
        }
    }


    std::cout << "-------------Sizes------------------------------------\n";

    std::cout << sizeof(xy_pts)/sizeof(*xy_pts) << std::endl;
    std::cout << xy_pts_array_ptr->size() << std::endl;
    std::cout << sizeof(xy_pts_2d)/sizeof(**xy_pts_2d) << std::endl;


    std::cout << "-------------Randomize------------------------------------\n";

    std::srand(time(NULL));

    const int MAX = 10;
    std::vector<float> float_val(MAX), float_shuffle(MAX);
    std::vector<float>::iterator it = float_val.begin(), it_shuffle = float_shuffle.begin();

    for ( ; it < float_val.end(); it++ ) {
        *it = rand()%100;
    }

    float_shuffle = float_val;

    std::random_shuffle(float_shuffle.begin(), float_shuffle.end());
    it = float_val.begin();
    it_shuffle = float_shuffle.begin();

    for ( ; it < float_val.end(); ) {
        printf("Normal %f, Random %f\n" ,*it++, *it_shuffle++);
    }


}