char pad_get_key(){
    if (Serial1.available() > 0) {
        char c = Serial1.read();
        Serial.println(c);
        switch (c) {
            case '0':
                return '0';
                break;
            case '1':
                return '1';
                break;
            case '2':
                return '2';
                break;
            case '3':
                return '3';
                break;
            case '4':
                return '4';
                break;
            case '5':
                return '5';
                break;
            case '6':
                return '6';
                break;
            case '7':
                return '7';
                break;
            case '8':
                return '8';
                break;
            case '9':
                return '9';
                break;
            case 'A':
                return 'A';
                break;
            case 'B':
                return 'B';
                break;
            case 'C':
                return 'C';
                break;
            case 'D':
                return 'D';
                break;
            case '*':
                return '*';
                break;
            case '#':
                return '#';
                break;
            default:
                return 0;
        }
    }
    return 0;
}