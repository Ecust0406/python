#! coding:utf-8
import os

def transform(input_dir, output_dir):
    filenames = os.listdir(input_dir)
    filenames.sort()
    for filename in filenames:
        print filename
        if filename[-1] == "i":
            avi_name = os.path.join(input_dir, filename)
            mp4_name = os.path.join(output_dir, filename[:-4] + ".mp4")
            cmd = " ".join(["avconv", "-i",
                            avi_name, "-c:v", "libx264", "-c:a", "copy", mp4_name])

            print cmd
            os.system(cmd)
    return


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="get input argument")
    parser.add_argument('-i', '--input_root', required=True,
                        help='input directory which contains origin raw_root')
    parser.add_argument('-o', '--output_root', required=True,
                        help='output directory which contains result video')
    args = parser.parse_args()
    while True:
        transform(args.input_root, args.output_root)
        exit()