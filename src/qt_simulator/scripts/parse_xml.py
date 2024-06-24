import xml.etree.ElementTree as ET


def parse_xml_file(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()

    time = []
    pos_data = {
        'HeadYaw': [],
        'HeadPitch': [],
        'RightElbowRoll': [],
        'RightShoulderPitch': [],
        'RightShoulderRoll': [],
        'LeftElbowRoll': [],
        'LeftShoulderPitch': [],
        'LeftShoulderRoll': []
    }

    for point in root.iter('point'):
        for key, val in pos_data.items():
            if point.find(key) is not None:
                val.append(float(point.find(key).text))
        time.append(point.get('time'))

    time_numeric = [int(numeric_string) for numeric_string in time]
    time_difference = [time_numeric[i + 1] - time_numeric[i] for i in range(len(time_numeric) - 1)]
    time_difference.append(0)

    return time_difference, pos_data


if __name__ == '__main__':
    FILE_PATH = '../../../resources/gestures/QT/hi.xml'
    time, data = parse_xml_file(FILE_PATH)