#subparsers = parser.add_subparsers(title='subcommands', description='')
#parser_opencv = subparsers.add_parser('opencv', help='builds or cleans opencv')
#parser_opencv.set_defaults(func=enter_opencv_fn)

group_opencv = parser.add_mutually_exclusive_group()

PROJECT_PWD = SOURCE_DIR + "/project/VirtualTestDriveFramework"
# export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig
# enter_caffe_fn
# enter_libsvm_fn
# enter_ivt_fn
# enter_external_algorithm_fn

