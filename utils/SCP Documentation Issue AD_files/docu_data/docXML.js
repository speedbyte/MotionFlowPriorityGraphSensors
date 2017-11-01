
function showOne( ID ) {
    document.getElementById( ID ).style.display = "block";
    return false;
}

function hideOne( ID ) {
    document.getElementById( ID ).style.display = "none";
    return false;
}

function tdWidth() {
    // TH_NUM is number of header columns
    var TH_NUM = document.getElementById('head1').getElementsByTagName('th').length;

    // read width of each column
    for ( var i = 0; i < TH_NUM; ++i ) {
        var WIDTH = document.getElementById('head1').getElementsByTagName('th')[ i ].offsetWidth;

        // set column width in both tables, to ensure equality
        document.getElementById('head2').getElementsByTagName('th')[ i ].style.width = WIDTH + 'px';
        //document.getElementById('head1').getElementsByTagName('th')[ i ].style.width = WIDTH + 'px';
    }
}

function windowWidth() {
    if ( window.innerWidth ) {
        return window.innerWidth;
    }
    else if ( document.body && document.body.offsetWidth ) {
        return document.body.offsetWidth;
    }
    else {
        return 0;
    }
}

function windowHeight() {
    if ( window.innerHeight ) {
        return window.innerHeight;
    }
    else if ( document.body && document.body.offsetHeight ) {
        return document.body.offsetHeight;
    }
    else {
        return 0;
    }
}

function rebuild() {
    if ( winWidth != windowWidth() || winHeight != windowHeight() ) {
        location.href = location.href;
    }
    window.setTimeout("tdWidth()", 500);
}
