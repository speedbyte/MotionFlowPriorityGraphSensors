INI_SEC(Global,"Global parameters");
INI_DBL(Alpha,          100.,"Weight of regularization term vs. data term");
INI_DBL(AlphaOF,        -1.,"Weight of regularization term vs. data term for optical flow (default is Alpha)");
INI_DBL(AlphaST,        -1., "Weight of regularization term vs. data term for stereo (default is Alpha)");
INI_DBL(AlphaSF,        -1.,"Weight of regularization term vs. data term for scene flow (default is Alpha)");
INI_DBL(Gamma,          1.5,"Weight of image gradients versus intensity in data term");
INI_DBL(Lambda,         2.0,"Weight of disparity flow versus optical flow in regularization");
INI_DBL(Mu,             0.1, "Weight of disparity versus optical flow in regularization");
INI_INT(MaxIterOuter,   30,"Max number of outer loop iterations");
INI_DBL(EpsilonOuter, 0.01,"Stop criterion for the first fixed point iteration");
INI_INT(MaxIterInner, 15,"Max number of inner loop iterations");
INI_DBL(EpsilonInner, 0.05,"Stop criterion for the second fixed point iteration in optical flow and scene flow computation");
INI_INT(MaxIterSOR,      10,"Max number of SOR iterations");
INI_INT(MaxIterSORSF,    20,"Max number of SOR iterations for Scene Flow");
INI_DBL(EpsilonSOR,   0.01,"Stop criterion for SOR for Stereo and Optic Flow");
INI_DBL(EpsilonSORSF,   0.01,"Stop criterion for SOR for Scene Flow");
INI_DBL(OmegaSOROF,     1.8,"OmegaSOR value for optical flow");
INI_DBL(OmegaSORST,     1.6,"OmegaSOR value for stereo");
INI_DBL(OmegaSORSF,     1.6,"OmegaSOR value for scene flow");
INI_STR(StereoExecutable,"./bp-vision/stereo","Path to the executable for stereo initialization");
INI_INT(PyramidLevelOF,37,"First pyramid level for Optical Flow (>=PyramidLevelSF)");
INI_INT(PyramidLevelST,8,"First pyramid level for Stereo (>=PyramidLevelSF)");
INI_INT(PyramidLevelSF,8,"First pyramid level for SceneFlow");
INI_INT(PyramidLevelFinal,1,"Final pyramid level");
INI_DBL(PyramidEta,0.9,"Pyramid reduction factor");
INI_INT(PyramidFloatingPoint,0,"Non-zero for floating point images in the pyramid");
INI_STR(LogFile,"sf.log","Log file name");
INI_INT(DoStereo,1,"Non-zero for initial stereo optimization");
INI_INT(InitStereoBP,1,"Initialize the stereo using external BP-based stereo");
INI_INT(DoSceneFlow,1,"Non-zero for full scene flow optimization");
INI_STR(Il0_FileName,NULL,"Left image at time 0");
INI_STR(Ilt_FileName,NULL,"Left image at time t");
INI_STR(Ir0_FileName,NULL,"Right image at time 0");
INI_STR(Irt_FileName,NULL,"Right image at time t");
INI_STR(ImagePyramidFileSuffix,NULL,"Image pyramid file suffix");
INI_STR(Il0PyramidFilePrefix,NULL,"");
INI_STR(IltPyramidFilePrefix,NULL,"");
INI_STR(Ir0PyramidFilePrefix,NULL,"");
INI_STR(IrtPyramidFilePrefix,NULL,"");

INI_SEC(GroundTruth,"Ground truth data");
INI_INT(GT_Special,0,"1 if rotating ball, 2 if Yosemite, 0 for other cases");
INI_STR(GT_D0_FileName,NULL,"Ground truth for initial disparity");
INI_DBL(GT_D0_Scale,1.0,"Scale for ground truth initial disparity");
INI_DBL(GT_D0_Invalid,0.0,"Value for occluded areas");
INI_STR(GT_Dt_FileName,NULL,"Ground truth for disparity at t");
INI_DBL(GT_Dt_Scale,1.0,"Scale for ground truth disparity at t");
INI_DBL(GT_Dt_Invalid,0.0,"Value for occluded areas");
INI_STR(GT_U_FileName,NULL,"Ground truth for left X flow");
INI_DBL(GT_U_Scale,1.0,"Scale for ground truth left X flow");
INI_DBL(GT_U_Invalid,255,"Value for occluded areas");
INI_STR(GT_V_FileName,NULL,"Ground truth for left Y flow");
INI_DBL(GT_V_Scale,1.0,"Scale for ground truth left Y flow");
INI_DBL(GT_V_Invalid,255,"Value for occluded areas");
INI_STR(GT_D0_View_FileName,NULL,"Ground truth view for initial disparity");
INI_STR(GT_Dt_View_FileName,NULL,"Ground truth view for disparity at t");
INI_STR(GT_U_View_FileName,NULL,"Ground truth view for left X flow");
INI_STR(GT_V_View_FileName,NULL,"Ground truth view for left Y flow");
INI_STR(GT_OcclusionsDiscontinuities,NULL,"Occlusions and discontinuities map");

INI_SEC(OpticFlow,"Optical flow computation");
INI_STR(OF_I0_FileName,NULL,"");
INI_STR(OF_It_FileName,NULL,"");
INI_STR(OF_It_Warped_FileName,NULL,"");
INI_STR(OF_U_increment_FileName,NULL,"");
INI_STR(OF_V_increment_FileName,NULL,"");
INI_STR(OF_U_FileName,NULL,"");
INI_STR(OF_V_FileName,NULL,"");
INI_STR(OF_disactivated_FileName,NULL,"Image of disactivated pixels");
INI_STR(OF_U_Final_FileName,NULL,"Left X flow output from optical flow computation");
INI_STR(OF_V_Final_FileName,NULL,"Left Y flow output from optical flow computation");
INI_STR(OF_Ur_Final_FileName,NULL,"Right X flow output from optical flow computation");
INI_STR(OF_Vr_Final_FileName,NULL,"Right Y flow output from optical flow computation");
INI_STR(OF_FlowAngularError_FileName,NULL,"Left flow angular error");
INI_STR(OF_FlowAngularErrorOcclusions_FileName,NULL,"Left flow angular error without occlusions");
INI_STR(OF_OmegaFileError, NULL, "");
INI_STR(OF_OmegaFileIter, NULL, "");
INI_INT(OF_ZeroTopBorder,0,"Zero out top image border in OF increments");



INI_SEC(Stereo,"Stereo computation");
INI_STR(ST_Il0_FileName,NULL,"Left filename for external stereo (should be PGM)");
INI_STR(ST_Ir0_FileName,NULL,"Right filename for external stereo (should be PGM)");
INI_STR(ST_D0_Init_Middlebury_FileName,"D0_ST_init_Middlebury.pgm","Initial disparity estimation in Middlebury format (should be PGM)");
INI_STR(ST_D0_FileName,NULL,"Final disparity estimation from stereo");

INI_SEC(SceneFlow,"Scene flow computation");
INI_STR(SF_Dt_Init_FileName,NULL,"Output false color disparity image");
INI_STR(SF_U_increment_FileName,NULL,"");
INI_STR(SF_V_increment_FileName,NULL,"");
INI_STR(SF_D0_increment_FileName,NULL,"");
INI_STR(SF_Dt_increment_FileName,NULL,"");
INI_STR(SF_U_FileName,NULL,"");
INI_STR(SF_V_FileName,NULL,"");
INI_STR(SF_D0_FileName,NULL,"");
INI_STR(SF_Dt_FileName,NULL,"");
INI_STR(SF_disactivated_stereoinit_FileName,NULL,"Image of disactivated pixels for stereo at time t");
INI_STR(SF_disactivated_stereo_FileName,NULL,"Image of disactivated pixels for stereo at time t+1");
INI_STR(SF_disactivated_flowleft_FileName,NULL,"Image of disactivated pixels for left flow");
INI_STR(SF_disactivated_flowright_FileName,NULL,"Image of disactivated pixels for right flow");
INI_STR(SF_Il0_FileName,NULL,"");
INI_STR(SF_Ilt_FileName,NULL,"");
INI_STR(SF_Ir0_FileName,NULL,"");
INI_STR(SF_Irt_FileName,NULL,"");
INI_STR(SF_Ilt_Warped_FileName,NULL,"");
INI_STR(SF_Ir0_Warped_FileName,NULL,"");
INI_STR(SF_Irt_Warped_FileName,NULL,"");
INI_STR(SF_U_Final_FileName,NULL,"");
INI_STR(SF_V_Final_FileName,NULL,"");
INI_STR(SF_D0_Final_FileName,NULL,"");
INI_STR(SF_Dt_Final_FileName,NULL,"");
INI_STR(SF_D0_Final_Middlebury_FileName,NULL,"");
INI_STR(SF_FlowAngularError_FileName,NULL,"Left flow angular error");
INI_STR(SF_FlowAngularErrorOcclusions_FileName,NULL,"Left flow angular error without occlusions");
INI_STR(SF_OmegaFileError, NULL, "");
INI_STR(SF_OmegaFileIter, NULL, "");
INI_INT(SF_Steps,0,"0 if one step scene flow, 1 for D optimization , U,V,D' optimization and then U,V,D,D'");


INI_SEC(Yosemite,"Yosemite ground truth optical flow data");
INI_STR(YosCloudsPath,"./Images/Yosemite/yos-clouds","Path to the yos-clouds directory");
INI_STR(YosImagesPath,"./Images/Yosemite/yos-images","Path to the yos-images directory");
INI_STR(YosFlowsPath,"./Images/Yosemite/yos-flows","Path to the yos-flows directory");
INI_STR(YosOutputSuffix,"png","Extension for the Yosemite converted images");
INI_STR(YosOutputCloudsPrefix,"./Images/Yosemite/YosemiteCloudsL.","Prefix to the output yos-clouds converted images directory (must exist)");
INI_STR(YosOutputImagesPrefix,"./Images/Yosemite/YosemiteL.","Prefix to the output  yos-images converted images directory (must exist)");
INI_STR(YosOutputFlowsPrefix,"./Images/Yosemite/GroundTruth","Prefix to the output  yos-images converted images directory (must exist)");