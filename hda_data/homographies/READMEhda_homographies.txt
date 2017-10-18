- "hda_homographies" - Contains one text file per camera with a 3x3 matrix representing the projection from the camera image plane to the ground plane.

Each text file can easily be loaded into matlab with: 
	"Homography = dlmread(filename)"

The groud plane reference frame for each floor is given by the two maps "7th_floor_ground_plane_reference_frame_map" and "8th_floor_ground_plane_reference_frame_map" (the *.fig files are matlab figures of these maps where the "Data Cursor" can be used to inspect the maps in detail).

