import xml.etree.ElementTree as ET


def parse_xml_file(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()

    time = []
    data = {
        'RightElbowRoll': [],
        'RightShoulderPitch': [],
        'RightShoulderRoll': [],
        'LeftElbowRoll': [],
        'LeftShoulderPitch': [],
        'LeftShoulderRoll': []
    }

    for point in root.iter('point'):
        for key, val in data.items():
            if point.find(key) is not None:
                val.append(float(point.find(key).text))
        time.append(point.get('time'))

    print(time)
    print(data)


if __name__ == '__main__':
    FILE_PATH = '../../../resources/gestures/QT/send_kiss.xml'
    parse_xml_file(FILE_PATH)
