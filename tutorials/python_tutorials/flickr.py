import flickrapi
import subprocess

api_key = 'e224418b91b4af4e8cdb0564716fa9bd'
api_secret = '7cddb9c9716501a0'
userid = '7438066@N04'

flickr = flickrapi.FlickrAPI(api_key, api_secret, format='parsed-json')
flickr.authenticate_via_browser(perms='read')
print flickr.token_cache.token


photos = flickr.photos.search(user_id=userid, per_page='100') # , extras='url_c'
sets = flickr.photosets.getList(user_id=userid)

#print photos
#print sets

total_photos=0
for photoset_index in sets['photosets']['photoset']:
    photoset_title = photoset_index['title']['_content']
    number_photos = photoset_index['photos']
    id = photoset_index['id']
    total_photos=total_photos+number_photos
    #subprocess.check_call(["bash", "-c", "flickr_download", "-k", api_key, "-s", api_secret, "-t -d", id])
    print "flickr_download -k " + api_key + " -s " + api_secret + " -t -d " + id
    print id,photoset_title,number_photos
total_number_of_albums = len(sets['photosets']['photoset'])
print total_number_of_albums

for photos_index in photos['photos']['photo']:
    photo_name = photos_index['title']
    photo_id = photos_index['id']

    #print photo_name, photo_id
total_number_of_pics = photos['photos']['total']
print total_photos, total_number_of_pics

flickr_etree = flickrapi.FlickrAPI(api_key, api_secret)
flickr_etree.authenticate_via_browser(perms='read')
print flickr_etree.token_cache.token


photos = flickr_etree.photos.search(user_id=userid, per_page='100') # , extras='url_c'
sets = flickr_etree.photosets.getList(user_id=userid)

set0 = sets.find('photosets').findall('photoset')[0]
print set0


#print photos
#print sets

#python flickrtouchr-album.py --user 7438066@N04 --album 72157668871273546 /media/veikas/NIKON$
#python flickrtouchr-album.py --user 7438066@N04 /media/veikas/NIKON$

# u'date_update': u'1502103278', u'visibility_can_see_set': 1, u'description': {u'_content': u''}, u'videos': u'0',
# u'title': {u'_content': u'India-2016-17'}, u'farm': 5, u'needs_interstitial': 0, u'primary': u'36422245315',
# u'server': u'4351', u'date_create': u'1502103183', u'photos': 191, u'secret': u'43ccf60ffb', u'count_comments': u'0',
# u'count_views': u'0', u'can_comment': 1, u'id': u'72157684645962234'




#old"e224418b91b4af4e8cdb0564716fa9bd","","72157685481201331-444770be0e04d2ec","cdfe78530c6f68bc","read","Veikas","Veikas","7438066@N04"
#new"e224418b91b4af4e8cdb0564716fa9bd","","72157685481201331-444770be0e04d2ec","cdfe78530c6f68bc","read","Veikas","Veikas","7438066@N04"

#nsid = u'7438066@N04'
#token = u'72157685481201331-444770be0e04d2ec'

#token_new = u'72157688418002705-f2c380be80d6880e'
#token_secret=3359a0f260c1dbba
