import tqdm, tqdm.notebook
from pathlib import Path

# Set up tqdm for notebook-friendly progress bars
tqdm.tqdm = tqdm.notebook.tqdm  

# Import necessary modules from hloc
from hloc import extract_features, match_features, reconstruction, pairs_from_exhaustive
from hloc.visualization import plot_images, read_image
from hloc.utils import viz_3d

# Define the directory for images and outputs
dataset = Path('mapping')  # Replace 'robot1' with your dataset directory
outputs = Path('outputs/mapping')
sfm_pairs = outputs / 'pairs-sfm.txt'
features = outputs / 'features.h5'
matches = outputs / 'matches.h5'

# Configuration for feature extraction and matching
feature_conf = extract_features.confs['disk']
matcher_conf = match_features.confs['disk+lightglue']

# List to store all reference images
references = []

# Collect references from the dataset
refs = [str(p.relative_to(dataset)) for p in (dataset).iterdir()]
references.extend(refs)
print(f"{len(refs)} mapping images from {dataset.name}")
plot_images([read_image(dataset / r) for r in refs], dpi=25)

# Extract features from all reference images
extract_features.main(feature_conf, dataset, image_list=references, feature_path=features)

# Generate exhaustive pairs for SfM
pairs_from_exhaustive.main(sfm_pairs, image_list=references)

# Match features between image pairs
match_features.main(matcher_conf, sfm_pairs, features=features, matches=matches)

# Reconstruct the 3D model from the images, pairs, features, and matches
model = reconstruction.main(outputs / 'sfm', dataset, sfm_pairs, features, matches, image_list=references)

# Visualize the resulting 3D model
fig = viz_3d.init_figure()
viz_3d.plot_reconstruction(fig, model, color='rgba(255,0,0,0.5)', name="mapping", points_rgb=True)
fig.show()
