# -*- coding: utf-8 -*-

def load_calibration_data(calibration_txt):
    data_dict = {}
    with open(calibration_txt, 'rU') as f:
        for line in f.readlines():
            (key, val) = line.split()
            data_dict[key] = int(val)
    print("Successfully Loaded Calibration Data: ", data_dict)
    return data_dict


def save_calibration_data(calibration_txt, data_dict):
    with open(calibration_txt, 'w') as f:
        for key, values in data_dict.items():
            f.write(key + '\t' + str(values) + '\n')
    print("Successfully Saved Calibration Data: ", data_dict)


def load_panel_html(html_dir):
    with open(html_dir) as f:
        panel_html = f.read()
    return panel_html


if __name__ == '__main__':
    calibration_txt = 'calibration.txt'
    with open(calibration_txt, 'r') as f:
        lines = f.readlines()
        print(lines)
