
file_dir=$1

if [ -d "${file_dir}" ]; then
    echo "${file_dir} exists!"
    break
else
    echo "${file_dir} does not exists"
    exit
fi


cp -rv  ./ "$file_dir"
rm -rfv "$file_dir/install.sh"
