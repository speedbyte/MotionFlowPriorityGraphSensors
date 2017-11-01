
// encapsulating sorting call with list name and sort sequence
function sortTree( listId, sortDescending ) {

    if ( typeof listId == "string" ) {

        var list = document.getElementById( listId );

        // call sort function with root unordered list
        sortUnorderedList( list, sortDescending );
    }
}

// actual sorting with list as node and sort sequence
function sortUnorderedList( list, sortDescending ) {

    var vals = []; // this is the array that gets sorted

    for ( var i = 0, l = list.childNodes.length; i < l; ++i ) {

        // there are also (empty) text nodes we do not want to sort
        if ( list.childNodes[ i ].nodeName == "LI" ) {
            vals.push( list.childNodes[ i ] );

            // a list item does only have exactly zero or one child list
            var childList = list.childNodes[ i ].getElementsByTagName("UL")[ 0 ];
            if ( childList ) {
                // recursive call for child lists
                sortUnorderedList( childList, sortDescending );
            }

        }
    }

    // sort using own compare criterion
    vals.sort( sortCompare );

    if ( sortDescending ) {
        vals.reverse();
    }

    // inject sorted nodes at the right position of current list
    for ( var i = 0, l = vals.length; i < l; ++i ) {

        list.insertBefore( vals[ i ], list.childNodes[ i ] );
    }
}

// sorting compare criterion especially for list items with anchor child tags
function sortCompare( li1, li2 ) {

    var li1Text;
    var li2Text;

    for ( var i = 0, l = li1.childNodes.length; i < l; ++i ) {
        if ( li1.childNodes[ i ].nodeName == "A" ) {
            li1Text = li1.childNodes[ i ].firstChild.nodeValue.toLowerCase();
            break;
        }
    }

    for ( var i = 0, l = li2.childNodes.length; i < l; ++i ) {
        if ( li2.childNodes[ i ].nodeName == "A" ) {
            li2Text = li2.childNodes[ i ].firstChild.nodeValue.toLowerCase();
            break;
        }
    }

    // compare text strings inside anchor child tag of list items
    return li1Text.localeCompare( li2Text );
}

// install sorting on the 'sortBtnAlpha' button
window.onload = function() {

    var desc = false;

    document.getElementById("sortBtnAlpha").onclick = function() {
        sortTree("navigation", desc );
        desc = !desc;
        return false;
    }
}
