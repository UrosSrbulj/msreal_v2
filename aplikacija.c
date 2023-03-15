#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <unistd.h>


int main(void){
	
	
	FILE *fp;
	char *str;
	size_t broj_bajtova = 36;
	
	
	fp = fopen("/dev/matmul", "w");
	
	if(fp==NULL){
		
		printf("Nije moguce otvoriti /dev/matmul.\n");
		return -1;
		
	}
	sleep(0.5);
	printf("	********************************\n");
	printf("	**Upisujem n=2 u /dev/matmul.**\n");
	printf("	********************************\n");
	
	fputs("n=2",fp);
	
	if(fclose(fp)){
		
		printf("Problem pri zatvaranju /dev/matmul.\n");
		return -1;	
		
	}
	
	fp = fopen("/dev/matmul", "w");
	
	if(fp==NULL){
		
		printf("Nije moguce otvoriti /dev/matmul.\n");
		return -1;
		
	}
	sleep(0.5);
	printf("	*******************************\n");
	printf("	**Upisujem m=8 u /dev/matmul.**\n");
	printf("	*******************************\n");
	
	fputs("m=8",fp);
	
	if(fclose(fp)){
		
		printf("Problem pri zatvaranju /dev/matmul.\n");
		return -1;	
		
	}
	
	fp = fopen("/dev/matmul", "w");
	
	if(fp==NULL){
		
		printf("Nije moguce otvoriti /dev/matmul.\n");
		return -1;
		
	}
	sleep(0.5);
	printf("	*******************************\n");
	printf("	**Upisujem m=3 u /dev/matmul.**\n");
	printf("	*******************************\n");
	
	fputs("m=3",fp);
	
	if(fclose(fp)){
		
		printf("Problem pri zatvaranju /dev/matmul.\n");
		return -1;	
		
	}
	
	fp = fopen("/dev/matmul", "w");
	
	if(fp==NULL){
		
		printf("Nije moguce otvoriti /dev/matmul.\n");
		return -1;
		
	}
	sleep(0.5);
	printf("	*******************************\n");
	printf("	**Upisujem p=4 u /dev/matmul.**\n");
	printf("	*******************************\n");
	
	fputs("p=4",fp);
	
	
	if(fclose(fp)){
		
		printf("Problem pri zatvaranju /dev/matmul.\n");
		return -1;	
		
	}
	//*******ZAVRSEN UPIS DIMENZIJA MATRICA U MATMUL*********
	
	fp = fopen("/dev/bram_a", "w");
	
	if(fp==NULL){
		
		printf("Nije moguce otvoriti /dev/bram_a.\n");
		return -1;
		
	}
	
	sleep(0.5);
	printf("	****************************************\n");
	printf("	**Upisujem 1,2;3,4,5,6; u /dev/bram_a.**\n");
	printf("	****************************************\n");
	
	fputs("1,2;3,4,5,6;",fp);
	
	if(fclose(fp)){
		
		printf("Problem pri zatvaranju /dev/bram_a.\n");
		return -1;	
		
	}
	
	fp = fopen("/dev/bram_a", "w");
	
	if(fp==NULL){
		
		printf("Nije moguce otvoriti /dev/bram_a.\n");
		return -1;
		
	}
	
	sleep(0.5);
	printf("	****************************************\n");
	printf("	**Upisujem 1,2,3;4,5,6; u /dev/bram_a.**\n");
	printf("	****************************************\n");
	
	fputs("1,2,3;4,5,6;",fp);
	
	if(fclose(fp)){
		
		printf("Problem pri zatvaranju /dev/bram_a.\n");
		return -1;	
		
	}
	//********ZAVRSEN UPIS U BRAM_A************
	
	fp = fopen("/dev/bram_b", "w");
	
	if(fp==NULL){
		
		printf("Nije moguce otvoriti /dev/bram_b.\n");
		return -1;
		
	}
	sleep(0.5);
	printf("	**********************************************************\n");
	printf("	**Upisujem 1,2,3,4;5,6,5014,8;9,10,11,12; u /dev/bram_b.**\n");
	printf("	**********************************************************\n");
	
	fputs("1,2,3,4;5,6,5014,8;9,10,11,12;",fp);
	
	if(fclose(fp)){
		
		printf("Problem pri zatvaranju /dev/bram_b.\n");
		return -1;	
		
	}
	
	fp = fopen("/dev/bram_b", "w");
	
	if(fp==NULL){
		
		printf("Nije moguce otvoriti /dev/bram_b.\n");
		return -1;
		
	}
	sleep(0.5);
	printf("	*******************************************************\n");
	printf("	**Upisujem 1,2,3,4;5,6,7,8;9,10,11,12; u /dev/bram_b.**\n");
	printf("	*******************************************************\n");
	
	fputs("1,2,3,4;5,6,7,8;9,10,11,12;",fp);
	
	if(fclose(fp)){
		
		printf("Problem pri zatvaranju /dev/bram_b.\n");
		return -1;	
		
	}
	//********ZAVRSEN UPIS U BRAM_B************
	
	fp = fopen("/dev/matmul", "w");
	
	if(fp==NULL){
		
		printf("Nije moguce otvoriti /dev/matmul.\n");
		return -1;
		
	}
	sleep(0.5);
	printf("	**************************************\n");
	printf("	**Upisujem start=haha u /dev/matmul.**\n");
	printf("	**************************************\n");
	
	fputs("start=haha",fp);
	
	if(fclose(fp)){
		
		printf("Problem pri zatvaranju /dev/matmul.\n");
		return -1;	
		
	}
	
	fp = fopen("/dev/matmul", "w");
	
	if(fp==NULL){
		
		printf("Nije moguce otvoriti /dev/matmul.\n");
		return -1;
		
	}
	sleep(0.5);
	printf("	*****************************************\n");
	printf("	**Upisujem start=trigger u /dev/matmul.**\n");
	printf("	*****************************************\n");
	
	fputs("start=trigger",fp);
	
	if(fclose(fp)){
		
		printf("Problem pri zatvaranju /dev/matmul.\n");
		return -1;	
		
	}
	
	//*********POKRENUTNO MNOZENJE MATRICA************
	
	sleep(0.5);
	printf("	*************************\n");
	printf("	**Rezultat mnozenja je:**\n");
	printf("	*************************\n");
	
	fp = fopen("/dev/bram_c", "r");
	
	if(fp==NULL){
		
		printf("Nije moguce otvoriti /dev/matmul.\n");
		return -1;
		
	}
	
	str = (char *)malloc(broj_bajtova+1);
	getline(&str, &broj_bajtova, fp);
	
	if(fclose(fp)){
		
		printf("Problem pri zatvaranju /dev/matmul.\n");
		return -1;	
		
	}
	free(str);
	
	return 0;
}