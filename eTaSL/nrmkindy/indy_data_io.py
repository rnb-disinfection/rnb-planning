# -*- coding: utf-8 -*- 



HIGH = 1
LOW = 0
ANALOG_MAX = 65535
ANALOG_MIN = 0


class DIO(object):
    def __init__(self, i, v):
        if v not in range(2):
            raise ValueError("The DIO Value must only input HIGH or LOW!")
        else:
            self.index = i
            self.value = v

    def get(self):
        return self.index, self.value


class DIOSet(object):
    def __init__(self, *args):
        self.dio_list = list()
        if args is not None:
            self.append(*args)

    def append(self, *args):
        if args is None:
            raise ValueError("The parameter is none")

        for i in range(len(args)):
            if i % 2 == 0:
                # 짝수이고 int 인 경우
                if isinstance(args[i], int) and isinstance(args[i + 1], int):
                    dio = DIO(args[i], args[i + 1])
                    self.dio_list.append(dio)
                # 짝수이고 DIO 인 경우
                elif isinstance(args[i], DIO):
                    pass
                else:
                    raise TypeError("If the parameter is even, it must be entered as int type.")
        # DIO 인 경우(짝수, 홀수)
        if len(args) % 2 == 1 or isinstance(args[0], DIO):
            for i in range(len(args)):
                if isinstance(args[i], DIO):
                    self.dio_list.append(args[i])
                else:
                    raise TypeError("If the parameter is odd, it must be entered as DIO type.")

    def get(self):
        return self.dio_list


class AIO(object):
    def __init__(self, i, v):
        if v not in range(65535):
            raise ValueError("The AIO Value is not included range")
        else:
            self.index = i
            self.value = v

    def get(self):
        return self.index, self.value


class AIOSet(object):
    def __init__(self, *args):
        self.aio_list = list()
        if args is not None:
            self.append(*args)

    def append(self, *args):
        if args is None:
            raise ValueError("The parameter is none")

        for i in range(len(args)):
            if i % 2 == 0:
                # 짝수이고 int 인 경우
                if isinstance(args[i], int) and isinstance(args[i + 1], int):
                    aio = AIO(args[i], args[i + 1])
                    self.aio_list.append(aio)
                # 짝수이고 AIO 인 경우
                elif isinstance(args[i], AIO):
                    pass
                else:
                    raise TypeError("If the parameter is even, it must be entered as int type.")
        # AIO 인 경우(짝수, 홀수)
        if len(args) % 2 == 1 or isinstance(args[0], AIO):
            for i in range(len(args)):
                if isinstance(args[i], AIO):
                    self.aio_list.append(args[i])
                else:
                    raise TypeError("If the parameter is odd, it must be entered as DIO type.")

    def get(self):
        return self.aio_list

# def create_dio_set