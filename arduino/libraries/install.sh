while read -ep "Enter arduino folder path location: " file_dir; do
    if [ -d "${file_dir}" ]; then
         echo "${file_dir} exists!"
         break
    else
        echo "${file_dir} does not exists - Enter right path: "
    fi
done

cp -rv  ./ $file_dir
rm -rfv $file_dir/install.sh
