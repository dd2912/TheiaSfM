#!/bin/bash

#module load apptainer/1.1.5

#apptainer run /work/pi_elearned_umass_edu/dave/teia-init/theia2.sif
#apptainer run /work/pi_elearned_umass_edu/dave/teia-init/theia.sif

#  ./run.sh <path/to/dataset/directory/buss> <name_of_experiment>
#
#

# Check the number of arguments
if [ "$#" -ne 2 ]; then
  echo "Usage: $0 dir exp_name"
  exit 1
fi

# Get the directory passed
main_directory="$1"

# Remove trailing slashes from the path
path="${main_directory%/}"
# Extract the dataset directory
dataset="${path##*/}"
# Extract the parent directory part
parent_dir="${path%$dataset}"

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Check if the directory exists
if [ -d "$main_directory" ]; then
  # List all subdirectories
  subdirectories=($(find "$main_directory" -type d))
  
  # Loop through the subdirectories
  for subdir in "${subdirectories[@]}"; do
    # Skip the main directory itself
    if [ "$subdir" != "$main_directory" ]; then
      
      # Get sequence name
      seq="${subdir##*/}"
      ss="${parent_dir%/}"

      # # skip if seq has aldready been done
      # echo "$ss/saved_exp/$dataset/$2/$seq"
      # if [ -d "$ss/saved_exp/$dataset/$2/$seq" ]; then
      #   echo "Directory $ss/saved_exp/$2/$seq exists. Continuing to the next loop iteration..."
      #   continue
      # fi

      # go back to scrip dir and execute python
      cd "$script_dir" || exit 1
      python3 create_flag.py -r "$parent_dir" -d "$dataset" -s "$seq" -t "$ss/saved_exp" -e "$2" || exit 1

      
      echo "Entering $subdir"
      # Change to the subdirectory
      cd "$subdir" || exit 1
      /home/fabien/Documents/TheiaSfM/build/bin/build_reconstruction --flagfile=./flag.txt || continue
      
      cd "$script_dir" || exit 1
      #now go back to script dir and convert pose
      python3 pose2tum.py --folder "$parent_dir/saved_exp/$dataset/$2/$seq" || exit 1
      
      # Return to the main directory
      cd "$main_directory" || exit 1
    fi
  done
else
  echo "Directory $main_directory does not exist."
fi
