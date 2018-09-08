//
// Created by veikas on 01.02.18.
//

#include "FlowImageExtended.h"



// interpolate all missing (=invalid) optical flow vectors
void FlowImageExtended::interpolateBackground () {

    // for each row do
    for (int32_t v=0; v<height_; v++) {

        // init counter
        int32_t count = 0;

        // for each pixel do
        for (int32_t u=0; u<width_; u++) {

            // if flow is valid. valid flows contains unsigned integers except 65535.
            // -1 is preserved for no flow detected
            // 65535 is default value of the flow frame.
            if ( (getObjectId(u,v) != -1) && (getObjectId(u,v) != 65535) ) {

                // at least one pixel requires interpolation
                if (count>=1) {

                    // first and last value for interpolation
                    int32_t u1 = u-count;
                    int32_t u2 = u-1;

                    float val = getObjectId(u1-1,v);

                    // set pixel to min flow
                    if (u1>0 && u2<width_-1) {
                        float fu_ipol = std::min(getFlowU(u1-1,v),getFlowU(u2+1,v));
                        float fv_ipol = std::min(getFlowV(u1-1,v),getFlowV(u2+1,v));
                        for (int32_t u_curr=u1; u_curr<=u2; u_curr++) {
                            setFlowU(u_curr,v,fu_ipol);
                            setFlowV(u_curr,v,fv_ipol);
                            setObjectId(u_curr,v,val);
                        }
                    }
                }

                // reset counter
                count = 0;

                // otherwise increment counter
            } else if ( getObjectId(u,v) == -1) {
                count++;
            }
        }

        /*
        // extrapolate to the left
        for (int32_t u=0; u<width_; u++) {
            if ((getObjectId(u,v) != -1) && (getObjectId(u,v) != 65535)) {
                for (int32_t u2=0; u2<u; u2++) {
                    setFlowU(u2,v,getFlowU(u,v));
                    setFlowV(u2,v,getFlowV(u,v));
                    setValid(u2,v,true);
                }
                break;
            }
        }


        // extrapolate to the right
        for (int32_t u=width_-1; u>=0; u--) {
            if ((getObjectId(u,v) != -1) && (getObjectId(u,v) != 65535)) {
                for (int32_t u2=u+1; u2<=width_-1; u2++) {
                    setFlowU(u2,v,getFlowU(u,v));
                    setFlowV(u2,v,getFlowV(u,v));
                    setValid(u2,v,true);
                }
                break;
            }
        }*/
    }

    /*

    // for each column do
    for (int32_t u=0; u<width_; u++) {

        // extrapolate to the top
        for (int32_t v=0; v<height_; v++) {
            if ((getObjectId(u,v) != -1) && (getObjectId(u,v) != 65535)) {
                for (int32_t v2=0; v2<v; v2++) {
                    setFlowU(u,v2,getFlowU(u,v));
                    setFlowV(u,v2,getFlowV(u,v));
                    setValid(u,v2,true);
                }
                break;
            }
        }

        // extrapolate to the bottom
        for (int32_t v=height_-1; v>=0; v--) {
            if ((getObjectId(u,v) != -1) && (getObjectId(u,v) != 65535)) {
                for (int32_t v2=v+1; v2<=height_-1; v2++) {
                    setFlowU(u,v2,getFlowU(u,v));
                    setFlowV(u,v2,getFlowV(u,v));
                    setValid(u,v2,true);
                }
                break;
            }
        }
    }
     */
}
