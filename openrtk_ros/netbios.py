import socket, struct
import time

class netbios_query:
    def __init__(self,name):
        self.name = name
        self.populate()
    def populate(self):
        self.HOST = '192.168.20.255'
        self.PORT = 137
        self.nqs = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.nqs.setblocking(False)
        self.nqs.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        self.network = '<broadcast>'
        self.QueryData = [
        b"\xa9\xfb",  # Transaction ID
        b"\x01\x10",  # Flags Query
        b"\x00\x01",  # Question:1
        b"\x00\x00",  # Answer RRS
        b"\x00\x00",  # Authority RRS
        b"\x00\x00",  # Additional RRS
        b"\x20",      # length of Name:32
        b"NAME",      # Name   
        b"\x00",      # NameNull
        b"\x00\x20",  # Query Type:NB
        b"\x00\x01"] # Class
        self.QueryData[7] = str.encode(self.netbios_encode(self.name))


    
    def netbios_encode(self,src):
        src = src.ljust(15,"\x20")
        src = src.ljust(16,"\x00")
        print(len(src))
        names = []
        for c in src:
            char_ord = ord(c)
            high_4_bits = char_ord >> 4
            low_4_bits = char_ord & 0x0f
            names.append(high_4_bits)
            names.append(low_4_bits)
            res = ""    
        for name in names:
            res += chr(0x41+name)
        return res

    def Query(self):
        wait_count = 10
        send_data = []
        ret = False
        for bytes_ele in self.QueryData:
            for list_ele in bytes_ele:
                send_data.append(list_ele)
        while wait_count:
            try:
                print(send_data)
                #self.nqs.sendto(bytes(send_data), (self.HOST, self.PORT))
                
                self.nqs.sendto(bytes(send_data), (self.network, self.PORT))
                data_rev, ADDR = self.nqs.recvfrom(1024)
                print('test')
                if(len(data_rev) > 0):
                    print(data_rev)
                    ret = True
                    break
            except:
                time.sleep(1)
                wait_count-= 1
        self.nqs.close()
        return ret

if __name__ == "__main__":
    nbns = netbios_query("OPENRTK")
    ret = nbns.Query()
    print(ret)
