//
// Created by veikas on 28.01.18.
//

#include "ObjectImageShapeData.h"
#include "ObjectPosition.h"


void Rectangle::process() {

    m_data.create(m_objectHeight, m_objectWidth, CV_8UC3);
}
