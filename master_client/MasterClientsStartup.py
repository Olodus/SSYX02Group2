

if __name__ == '__main__':

    # Builds the standard Master Client
    b = MasterBuilder()
    d = StdMasterDirector(b)
    d.construct()
    master = b.getClient()

    # Runs the masterclient
    master.run()
