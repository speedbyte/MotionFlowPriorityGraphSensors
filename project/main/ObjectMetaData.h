//
// Created by veikas on 21.02.18.
//

#ifndef MAIN_OBJECTMETADATA_H
#define MAIN_OBJECTMETADATA_H


/*
%YAML:1.0
iterationNr: 100
strings:
   - "image1.jpg"
   - Awesomeness
   - "baboon.jpg"
Mapping:
   One: 1
   Two: 2
R: !!opencv-matrix
   rows: 3
   cols: 3
   dt: u
   data: [ 1, 0, 0, 0, 1, 0, 0, 0, 1 ]
T: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [ 0., 0., 0. ]
MyData:
   A: 97
   X: 3.1415926535897931e+000
   id: mydata1234
*/


class ObjectMetaData {

    ObjectImageShapeData m_objectMetaData_shape;
    ObjectSceneGroundTruth m_objectMetaData_position;

    std::string m_objectMetaData_name;
    ushort m_objectMetaData_startPoint;


public:

    ObjectMetaData() {};
    ObjectMetaData(ObjectImageShapeData shape, ObjectSceneGroundTruth position, std::string name, ushort startPoint) :
    m_objectMetaData_shape(shape), m_objectMetaData_position(position), m_objectMetaData_name(name), m_objectMetaData_startPoint(startPoint) {} ;

    void fillData() {

    }

    ObjectImageShapeData& getObjectShape() {
        return m_objectMetaData_shape;
    }

    ObjectSceneGroundTruth& getObjectPixelPosition() {
        return m_objectMetaData_position;
    }

    std::string& getObjectName() {
        return m_objectMetaData_name;
    }

    ushort& getObjectStartPoint() {
        return m_objectMetaData_startPoint;
    }

    void setObjectName(std::string objectName) {
        m_objectMetaData_name = objectName;
    }

    void setObjectShape(ObjectImageShapeData objectShape) {
        m_objectMetaData_shape = objectShape;
    }

};


#endif //MAIN_OBJECTMETADATA_H
