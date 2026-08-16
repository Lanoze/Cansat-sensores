#include "sd_read_write.h" //[cite: 2]

void listDir(SdFs &fs, const char * dirname, uint8_t levels){ //[cite: 2]
  Serial.printf("Listing directory: %s\n", dirname); //[cite: 2]

  FsFile root = fs.open(dirname); //[cite: 2]
  if(!root){ //[cite: 2]
    Serial.println("Failed to open directory"); //[cite: 2]
    return; //[cite: 2]
  }
  if(!root.isDirectory()){ //[cite: 2]
    Serial.println("Not a directory"); //[cite: 2]
    return; //[cite: 2]
  }

  FsFile file = root.openNextFile(); //[cite: 2]
  while(file){ //[cite: 2]
    char filename[64];
    file.getName(filename, sizeof(filename));
    
    if(file.isDirectory()){ //[cite: 2]
      Serial.print("  DIR : "); //[cite: 2]
      Serial.println(filename); //[cite: 2]
      if(levels){ //[cite: 2]
        listDir(fs, filename, levels -1); //[cite: 2]
      }
    } else { //[cite: 2]
      Serial.print("  FILE: "); //[cite: 2]
      Serial.print(filename); //[cite: 2]
      Serial.print("  SIZE: "); //[cite: 2]
      Serial.println(file.size()); //[cite: 2]
    }
    file = root.openNextFile(); //[cite: 2]
  }
}

void createDir(SdFs &fs, const char * path){ //[cite: 2]
  if(fs.mkdir(path)){ //[cite: 2]
    Serial.println("Dir created"); //[cite: 2]
  }
}

void removeDir(SdFs &fs, const char * path){ //[cite: 2]
  if(fs.rmdir(path)){ //[cite: 2]
    Serial.println("Dir removed"); //[cite: 2]
  }
}

void readFile(SdFs &fs, const char * path){ //[cite: 2]
  FsFile file = fs.open(path, O_READ); //[cite: 2]
  if(!file){ //[cite: 2]
    return; //[cite: 2]
  }
  while(file.available()){ //[cite: 2]
    Serial.write(file.read()); //[cite: 2]
  }
  file.close(); //[cite: 2]
}

void writeFile(SdFs &fs, const char * path, const char * message){ //[cite: 2]
  FsFile file = fs.open(path, O_WRITE | O_CREAT | O_TRUNC); //[cite: 2]
  if(!file){ //[cite: 2]
    return; //[cite: 2]
  }
  file.print(message); //[cite: 2]
  file.close(); //[cite: 2]
}

void appendFile(SdFs &fs, const char * path, const char * message){ //[cite: 2]
  FsFile file = fs.open(path, O_WRITE | O_CREAT | O_APPEND); //[cite: 2]
  if(!file){ //[cite: 2]
    return; //[cite: 2]
  }
  file.print(message); //[cite: 2]
  file.close(); //[cite: 2]
}

void renameFile(SdFs &fs, const char * path1, const char * path2){ //[cite: 2]
  fs.rename(path1, path2); //[cite: 2]
}

void deleteFile(SdFs &fs, const char * path){ //[cite: 2]
  fs.remove(path); //[cite: 2]
}