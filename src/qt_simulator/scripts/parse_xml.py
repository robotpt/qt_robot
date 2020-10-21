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

    return time, data

if __name__ == '__main__':
    FILE_PATH = '../../../resources/gestures/QT/hi.xml'
    time, data = parse_xml_file(FILE_PATH)
    # print(len(time))
    # state = 0
    # if len(data['RightShoulderPitch']) is not 0:
    #     right_shoulder_pitch = data['RightShoulderPitch'][state]
    # else:
    #     right_shoulder_pitch = 0
    #
    # if len(data['RightShoulderRoll']) is not 0:
    #     right_shoulder_roll = data['RightShoulderRoll'][state]
    # else:
    #     right_shoulder_roll = 0
    #
    # if len(data['RightElbowRoll']) is not 0:
    #     right_elbow_roll = data['RightElbowRoll'][state]
    # else:
    #     right_elbow_roll = 0
    #
    # print(right_elbow_roll)
    # print(right_shoulder_pitch)
    # print(right_shoulder_roll)