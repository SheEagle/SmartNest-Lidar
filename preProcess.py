import datetime


def pre_process(before_filename_S, before_filename_X):
    temp_filename_S = before_filename_S.rsplit('_', 1)[0] + '_temp.txt'
    temp_filename_X = before_filename_X.rsplit('_', 1)[0] + '_temp.txt'
    after_filename_S = before_filename_S.rsplit('_', 1)[0] + '_after.txt'
    after_filename_X = before_filename_X.rsplit('_', 1)[0] + '_after.txt'

    """将S、X的背景文件数据进行时间戳上的对齐"""
    with open(before_filename_S, "r") as file_S:
        # Read the contents of the file line by line
        lines_S = file_S.readlines()

    with open(temp_filename_S, "w") as temp_file_S:
        for i in range(0, int((len(lines_S) - 1) / 2)):
            temp_file_S.write(lines_S[2 * i])
            temp_file_S.write(lines_S[2 * i + 1])

    with open(before_filename_X, "r") as file_X:
        # Read the contents of the file line by line
        lines_X = file_X.readlines()

    with open(temp_filename_X, "w") as temp_file_X:
        for i in range(0, int((len(lines_X) - 1) / 2)):
            temp_file_X.write(lines_X[2 * i])
            temp_file_X.write(lines_X[2 * i + 1])

    with open(temp_filename_S, "r") as temp_file_S:
        lines_S = temp_file_S.readlines()

    with open(temp_filename_X, "r") as temp_file_X:
        lines_X = temp_file_X.readlines()

    """基于时间戳的数据对齐"""
    base_lines = lines_S
    base_file_name = after_filename_S
    other_lines = lines_X
    other_file_name = after_filename_X
    if len(lines_S) > len(lines_X):                                                                                     #数量多的是其他文件，用来配准数量少的，temp暂存在时间范围内数据，after保存最后数据
        base_lines = lines_X
        base_file_name = after_filename_X
        other_lines = lines_S
        other_file_name = after_filename_S

    with open(base_file_name, "w") as base_file:
        with open(other_file_name, "w") as other_file:
            for i in range(0, int((len(base_lines) - 1) / 2)):
                timestamp_base = datetime.datetime.strptime(base_lines[2 * i].replace('\n', '').replace('\r', ''),
                                                            "%Y-%m-%d %H:%M:%S.%f")
                timestamp_other = datetime.datetime.strptime(other_lines[2 * i].replace('\n', '').replace('\r', ''),
                                                             "%Y-%m-%d %H:%M:%S.%f")
                if abs((timestamp_base - timestamp_other).total_seconds() * 1000) < 50:  # 33/2                         #时间相差在一个范围界限内保存，不在一个就抛弃base,他数据多，匹配下一个
                    base_file.write(base_lines[2 * i])
                    base_file.write(base_lines[2 * i + 1])
                    other_file.write(other_lines[2 * i])
                    other_file.write(other_lines[2 * i + 1])
                else:
                    next_line = i + 1
                    while next_line < len(other_lines) / 2:
                        next_timestamp_other = datetime.datetime.strptime(other_lines[2 * next_line].replace('\n', '').
                                                                          replace('\r', ''), "%Y-%m-%d %H:%M:%S.%f")
                        if abs((timestamp_base - next_timestamp_other).total_seconds() * 1000) < 50:  # 33/2
                            base_file.write(base_lines[2 * i])
                            base_file.write(base_lines[2 * i + 1])
                            other_file.write(other_lines[2 * next_line])
                            other_file.write(other_lines[2 * next_line + 1])
                            break
                        else:
                            next_line = next_line + 1