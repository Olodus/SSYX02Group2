

if __name__ == '__main__':

    b = MasterBuilder()
    d = StdMasterDirector(b)
    d.construct()

    master = b.getClient()

    master.run()
