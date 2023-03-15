#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <linux/version.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>

#include <linux/uaccess.h>

#include <linux/io.h> //iowrite ioread
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>//ioremap


MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("Test Driver for MATRIX MULTIPLIER.");

#define DEVICE_NAME "matmul" 
#define DRIVER_NAME "matmul_driver"
#define BUFF_SIZE 250


//****************************** FUNCTION PROTOTYPES ****************************************//
static int matmul_probe(struct platform_device *pdev);
static int matmul_open(struct inode *i, struct file *f);
static int matmul_close(struct inode *i, struct file *f);
static ssize_t matmul_read(struct file *f, char __user *buf, size_t len, loff_t *off);
static ssize_t matmul_write(struct file *f, const char __user *buf, size_t length, loff_t *off);
static int __init matmul_init(void);
static void __exit matmul_exit(void);
static int matmul_remove(struct platform_device *pdev);

//***************************** GLOBAL VARIABLES *********************************************//

struct matmul_info {
  unsigned long mem_start;
  unsigned long mem_end;
  void __iomem *base_addr;
};

dev_t my_dev_id;
static struct class *my_class;
static struct device *my_device;
static struct cdev *my_cdev;
static int int_cnt;

static struct matmul_info *va = NULL;
static struct matmul_info *vb = NULL;
static struct matmul_info *vc = NULL;
static struct matmul_info *vm = NULL;

unsigned int koliko_unetih_brojeva_a;
unsigned int uneti_brojevi_a_reverse[BUFF_SIZE];
unsigned int uneti_brojevi_a[BUFF_SIZE];
unsigned int uneti_brojevi_a_matrix[8][8];

unsigned int koliko_unetih_brojeva_b;
unsigned int uneti_brojevi_b_reverse[BUFF_SIZE];
unsigned int uneti_brojevi_b[BUFF_SIZE];
unsigned int uneti_brojevi_b_matrix[8][8];

unsigned int koliko_unetih_brojeva_c;
unsigned int uneti_brojevi_c_reverse[BUFF_SIZE];
unsigned int uneti_brojevi_c[BUFF_SIZE];
unsigned int uneti_brojevi_c_matrix[8][8];


//*********************OPERACIJE NAD NODE FAJLOVIMA*************************************
static struct file_operations my_fops =
{
	.owner = THIS_MODULE,
	.open = matmul_open,
	.read = matmul_read,
	.write = matmul_write,
	.release = matmul_close,
};


//*********************POVEZUJE INFORMACIJE SA STABLOM UREĐAJA*************************************  
static struct of_device_id matmul_of_match[] = {
  { .compatible = "xlnx,axi-bram-ctrl-A", },
  { .compatible = "xlnx,axi-bram-ctrl-B", },
  { .compatible = "xlnx,axi-bram-ctrl-C", },
  { .compatible = "xlnx,matmul", },
  { /* end of list */ },
};

static struct platform_driver matmul_driver = {
  .driver = {
    .name = DRIVER_NAME,
    .owner = THIS_MODULE,
    .of_match_table	= matmul_of_match,
  },
  .probe		= matmul_probe,
  .remove	= matmul_remove,
};

MODULE_DEVICE_TABLE(of, matmul_of_match);


static int matmul_probe(struct platform_device *pdev)
{
  struct resource *r_mem;
  struct resource *r_mem2;
  struct resource *r_mem3;
  struct resource *r_mem4;
  int rc = 0;
  int rc2 = 0;
  int rc3 = 0;
  int rc4 = 0;
//----------------------------------------------------------
  if (((!va) && (!vb) && (!vc) && (!vm))){
  printk(KERN_INFO "Probing va\n");
  // Get phisical register adress space from device tree
  r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!r_mem) {
    printk(KERN_ALERT "matmul_probe: Failed to get reg resource\n");
    return -ENODEV;
  }
  // Get memory for structure vga_info
  va = (struct matmul_info *) kmalloc(sizeof(struct matmul_info), GFP_KERNEL);
  if (!va) {
    printk(KERN_ALERT "matmul_probe: Could not allocate timer device\n");
    return -ENOMEM;
  }
  // Put phisical adresses in timer_info structure
  va->mem_start = r_mem->start;
  va->mem_end = r_mem->end;
  
  printk(KERN_INFO "Start address:%x \t end address:%x", r_mem->start, r_mem->end);
    
  // Reserve that memory space for this driver
  if (!request_mem_region(va->mem_start,va->mem_end - va->mem_start + 1, DRIVER_NAME))
  {
    printk(KERN_ALERT "matmul_probe: Could not lock memory region at %p\n",(void *)va->mem_start);
    rc = -EBUSY;
    goto error1;
  }    
  // Remap phisical to virtual adresses

  va->base_addr = ioremap(va->mem_start, va->mem_end - va->mem_start + 1);
  if (!va->base_addr) {
    printk(KERN_ALERT "matmul_probe: Could not allocate memory\n");
    rc = -EIO;
    goto error2;
  }

  printk(KERN_NOTICE "matmul_probe: BRAM_CTRL_A uredjaj ubacen\n");
  goto kraj_probe;
 error2:
  release_mem_region(va->mem_start, va->mem_end - va->mem_start + 1);
 error1:
  return rc;
  }
  //-------------------------------------------------------------------------------
  else if(((va) && (!vb) && (!vc) && (!vm)))
  {
	  
  printk(KERN_INFO "Probing vb\n");
  // Get phisical register adress space from device tree
  r_mem2 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!r_mem2) {
    printk(KERN_ALERT "matmul_probe: Failed to get reg resource\n");
    return -ENODEV;
  }
  // Get memory for structure vga_info
  vb = (struct matmul_info *) kmalloc(sizeof(struct matmul_info), GFP_KERNEL);
  if (!vb) {
    printk(KERN_ALERT "matmul_probe: Could not allocate timer device\n");
    return -ENOMEM;
  }
  // Put phisical adresses in timer_info structure
  vb->mem_start = r_mem2->start;
  vb->mem_end = r_mem2->end;
  
  printk(KERN_INFO "Start address:%x \t end address:%x", r_mem2->start, r_mem2->end);
    
  // Reserve that memory space for this driver
  if (!request_mem_region(vb->mem_start,vb->mem_end - vb->mem_start + 1, DRIVER_NAME))
  {
    printk(KERN_ALERT "matmul_probe: Could not lock memory region at %p\n",(void *)vb->mem_start);
    rc2 = -EBUSY;
    goto greska1;
  }    
  // Remap phisical to virtual adresses

  vb->base_addr = ioremap(vb->mem_start, vb->mem_end - vb->mem_start + 1);
  if (!vb->base_addr) {
    printk(KERN_ALERT "matmul_probe: Could not allocate memory\n");
    rc2 = -EIO;
    goto greska2;
  }

  printk(KERN_NOTICE "matmul_probe: BRAM_CTRL_B uredjaj ubacen\n");
  goto kraj_probe;
 greska2:
  release_mem_region(vb->mem_start, vb->mem_end - vb->mem_start + 1);
 greska1:
  return rc2;
	  
  }
//--------------------------------------------------------------------------
 else if(((va) && (vb) && (!vc) && (!vm)))
  {
	  
  printk(KERN_INFO "Probing vc\n");
  // Get phisical register adress space from device tree
  r_mem3 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!r_mem3) {
    printk(KERN_ALERT "matmul_probe: Failed to get reg resource\n");
    return -ENODEV;
  }
  // Get memory for structure vga_info
  vc = (struct matmul_info *) kmalloc(sizeof(struct matmul_info), GFP_KERNEL);
  if (!vc) {
    printk(KERN_ALERT "matmul_probe: Could not allocate timer device\n");
    return -ENOMEM;
  }
  // Put phisical adresses in timer_info structure
  vc->mem_start = r_mem3->start;
  vc->mem_end = r_mem3->end;
  
  printk(KERN_INFO "Start address:%x \t end address:%x", r_mem3->start, r_mem3->end);
    
  // Reserve that memory space for this driver
  if (!request_mem_region(vc->mem_start,vc->mem_end - vc->mem_start + 1, DRIVER_NAME))
  {
    printk(KERN_ALERT "matmul_probe: Could not lock memory region at %p\n",(void *)vc->mem_start);
    rc3 = -EBUSY;
    goto greska3;
  }    
  // Remap phisical to virtual adresses

  vc->base_addr = ioremap(vc->mem_start, vc->mem_end - vc->mem_start + 1);
  if (!vc->base_addr) {
    printk(KERN_ALERT "matmul_probe: Could not allocate memory\n");
    rc3 = -EIO;
    goto greska4;
  }

  printk(KERN_NOTICE "matmul_probe: BRAM_CTRL_C uredjaj ubacen\n");
  goto kraj_probe;
 greska4:
  release_mem_region(vc->mem_start, vc->mem_end - vc->mem_start + 1);
 greska3:
  return rc3;
	  
  }
//---------------------------------------------------------------------------

  else if(((va) && (vb) && (vc) && (!vm)))
  {
	  
  printk(KERN_INFO "Probing vm\n");
  // Get phisical register adress space from device tree
  r_mem4 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!r_mem4) {
    printk(KERN_ALERT "matmul_probe: Failed to get reg resource\n");
    return -ENODEV;
  }
  // Get memory for structure vga_info
  vm = (struct matmul_info *) kmalloc(sizeof(struct matmul_info), GFP_KERNEL);
  if (!vm) {
    printk(KERN_ALERT "matmul_probe: Could not allocate timer device\n");
    return -ENOMEM;
  }
  // Put phisical adresses in timer_info structure
  vm->mem_start = r_mem4->start;
  vm->mem_end = r_mem4->end;
  
  printk(KERN_INFO "Start address:%x \t end address:%x", r_mem4->start, r_mem4->end);
    
  // Reserve that memory space for this driver
  if (!request_mem_region(vm->mem_start,vm->mem_end - vm->mem_start + 1, DRIVER_NAME))
  {
    printk(KERN_ALERT "matmul_probe: Could not lock memory region at %p\n",(void *)vm->mem_start);
    rc4 = -EBUSY;
    goto greska5;
  }    
  // Remap phisical to virtual adresses

  vm->base_addr = ioremap(vm->mem_start, vm->mem_end - vm->mem_start + 1);
  if (!vm->base_addr) {
    printk(KERN_ALERT "matmul_probe: Could not allocate memory\n");
    rc4 = -EIO;
    goto greska6;
  }

  printk(KERN_NOTICE "matmul_probe: matmul uredjaj ubacen\n");
  goto kraj_probe;//ALL OK
 greska6:
  release_mem_region(vm->mem_start, vm->mem_end - vm->mem_start + 1);
 greska5:
  return rc4;
	  
  }
  //-----------------------------------------------------------------
  kraj_probe:
  return 0; //ALL ok
}

//###################################  REMOVE ##################################################



static int matmul_remove(struct platform_device *pdev)
{
 
  if((request_mem_region(va->mem_start,va->mem_end - va->mem_start + 1, DRIVER_NAME)== NULL)){
	  
  printk(KERN_INFO "matmul_remove: va remove in process");
  iounmap(va->base_addr);
  release_mem_region(va->mem_start, va->mem_end - va->mem_start + 1);
  printk(KERN_INFO "matmul_remove: matmul_driver removed va");
  goto kraj1;
  
  }
  
  else if ((request_mem_region(vb->mem_start,vb->mem_end - vb->mem_start + 1, DRIVER_NAME)== NULL)){
	  
  printk(KERN_INFO "matmul_remove: vb remove in process");
  iounmap(vb->base_addr);
  release_mem_region(va->mem_start, va->mem_end - va->mem_start + 1);
  release_mem_region(vb->mem_start, vb->mem_end - vb->mem_start + 1);
  printk(KERN_INFO "matmul_remove: matmul_driver removed vb");
  goto kraj1;
  
  
  }
   
    else if ((request_mem_region(vc->mem_start,vc->mem_end - vc->mem_start + 1, DRIVER_NAME)== NULL)){
		
  printk(KERN_INFO "matmul_remove: vc remove in process");
  iounmap(vc->base_addr);
  release_mem_region(va->mem_start, va->mem_end - va->mem_start + 1);
  release_mem_region(vb->mem_start, vb->mem_end - vb->mem_start + 1);
  release_mem_region(vc->mem_start, vc->mem_end - vc->mem_start + 1);
  printk(KERN_INFO "matmul_remove: matmul_driver removed vc");
  goto kraj1;
  
  }
  
   else {
	   
  printk(KERN_INFO "matmul_remove: vm remove in process");
  iounmap(vm->base_addr);
  release_mem_region(va->mem_start, va->mem_end - va->mem_start + 1);
  release_mem_region(vb->mem_start, vb->mem_end - vb->mem_start + 1);
  release_mem_region(vc->mem_start, vc->mem_end - vc->mem_start + 1);
  release_mem_region(vm->mem_start, vm->mem_end - vm->mem_start + 1);
  printk(KERN_INFO "matmul_remove: matmul_driver removed vm");
  goto kraj2;
  
  }
  
  kraj2:
  kfree(va);
  kfree(vb);
  kfree(vc);
  kfree(vm);
  kraj1:
  return 0;
}

//******************************* OPEN  *************************************//
static int matmul_open(struct inode *i, struct file *f)
{
  printk(KERN_INFO "matmul opened\n");
  return 0;
}

//################################## CLOSE ########################################
static int matmul_close(struct inode *i, struct file *f)
{
  printk(KERN_INFO "matmul closed\n");
  return 0;
}

///################################### PROBE ###############################


ssize_t matmul_read(struct file *f, char __user *buffer, size_t length, loff_t *offset) 
{
    unsigned int ready_r,start_r,n_r,m_r,p_r;
    unsigned int vrednost;
    unsigned int petlja_brojac = 0;
    unsigned int broj_kolona, broj_redova;
    int minor;
  
    printk("matmul read\n");
    minor = MINOR(f->f_inode->i_rdev);
  
    if(minor == 0){ // citamo iz bram_a
	  
	    broj_kolona = ioread32(vm->base_addr + 12); // pročitaj m iz matmul
	    broj_redova = ioread32(vm->base_addr + 8);  // pročitaj n iz matmul
	  
	    for (petlja_brojac = 0;petlja_brojac < (broj_kolona*broj_redova) ; petlja_brojac = petlja_brojac + 1){
    		vrednost = ioread32(va->base_addr + 4*petlja_brojac);
		    printk(KERN_CONT "%u",vrednost);
		  
		    if((petlja_brojac+1)%broj_kolona == 0){
			    printk(KERN_CONT ";");
		        //printk("uspesno procitao va->base_addr + 4*%u\n", petlja_brojac);
		    }
		    else {
			    printk(KERN_CONT ",");
		    }
	    }
	    goto procitano;
    }

    procitano:
    return 0; 
}

//############################################### WRITE ##############################################################################3
ssize_t matmul_write(struct file *f, const char __user *buffer, size_t length, loff_t *offset) 
{
    char buff[BUFF_SIZE];
	char buff_pomocni[2];
	char buff_pomocni2[10];
	char n[20] = "n=";
	char m[20] = "m=";
	char p[20] = "p=";
	char start[20] = "start=";
	char osmica[2]="8";
	char devetka[2]="9";
	char jedan[2]="1";
	char nula[2]="0";
	char trig[10]="trigger";
    int minor;
    int ret;
    unsigned long long drugi;
	unsigned long drugi_manji;
	unsigned int manji_indeks, veci_indeks, koliko_cifara;
	unsigned long broj_ceo;
    unsigned int xpos,ypos;
	char *dimenzija_m;
	char *dimenzija_p;
	unsigned prava_dim_n = 0;
	unsigned prava_dim_m = 0;
	unsigned prava_dim_p = 0;
	unsigned razlika_adresa = 0;
	unsigned int broj_redova_a;
	unsigned int broj_kolona_b;
	unsigned int broj_kolona_a;
	unsigned pozicija_zareza[BUFF_SIZE];
	int i,k,brojac, upis;
	unsigned int u, q;
	char prvi[BUFF_SIZE];
	unsigned long long rgb;
	u = 0;
	i = 0;
	k = 0;
	//p = 0;
	brojac = 0;
	upis = 0;
   
    ret = 0;
    xpos = 0;
    ypos = 0;
    rgb = 0;


    printk("matmul write\n");
    minor = MINOR(f->f_inode->i_rdev);
    ret = copy_from_user(buff, p, length); 
    
    if(ret){
        printk("copy from user failed \n");
        return -EFAULT;
    } 
    buff[length] = '\0';

    printk(KERN_NOTICE"buff = %s",buff);

    if(minor == 0){ //ako upisujemo u bram_a
        
        prava_dim_m = 0;
	    dimenzija_m = strchr(buff, ';'); // trazi prvo pojavljivanje znaka ; u stringu buff i taj broj smesta u dimenzija_m
	  
	    brojac = 0; 
	    razlika_adresa = dimenzija_m - buff;
	  
	    for( i=0; i<razlika_adresa; i++ ){
	        if(buff[i]==','){
		        prava_dim_m = prava_dim_m + 1;
		    }
	    }

	    prava_dim_m = prava_dim_m + 1; // treba da bi se dobila tacna vrednost jer je na kraju reda uvek ; a ne ,
	    printk(KERN_WARNING "VGA_write: dimenzija_m = %d\n",prava_dim_m); //ovo je dimenzija dobijena iz unosa (ne iz matmula)
	  
	    for( i=0; i<length; i=i+1 ){
		    if(buff[i]==';'){
			    prava_dim_n = prava_dim_n + 1;
		    }
		}
	    printk(KERN_WARNING "VGA_write: dimenzija_n = %u\n",prava_dim_n); //ovo je dimenzija dobijena iz unosa (ne iz matmula)
	  
	    for(i=0 ; i<length; i=i+1){  //sluzi da smesti pozicije zareza  
		    if((buff[i]==',') || (buff[i]==';')){
			    pozicija_zareza[i] = i;
		    }
		    else{
			    pozicija_zareza[i] = 0;
		    }  
	    }

	    //**********FUNKCIJA ZA RACUNANJE BROJA CIFARA I KOJE SU TO CIFRE***********************
	    i = length - 1;
	    u = 0; // brojac za niz uneti_brojevi_b,a
	    
        while(i>=0){
		  
		    if(pozicija_zareza[i]!=0){
		
			    veci_indeks = pozicija_zareza[i];	
			
			    for(k = i-1; k >= 0; k = k - 1){
				
				    if((pozicija_zareza[k]!=0) || (k==0)){
					manji_indeks = pozicija_zareza[k];
					
					if(k==0){
						koliko_cifara = veci_indeks-manji_indeks;
					}
					else{
					    koliko_cifara = veci_indeks-manji_indeks-1;
					}
					
					switch(koliko_cifara){
						case 1:                
							prvi[0]=buff[i-1]; 
							prvi[1] = '\0';
							ret = kstrtoull(prvi, 0, &drugi);
							broj_ceo=drugi;
							break;
						case 2:
							prvi[0]=buff[i-2]; 
							prvi[1]=buff[i-1];
							prvi[2] = '\0';
							ret = kstrtoull(prvi, 0, &drugi);
							broj_ceo=drugi;							
							break;
						case 3:
							prvi[0]=buff[i-3]; 
							prvi[1]=buff[i-2];
							prvi[2]=buff[i-1];
							prvi[3] = '\0';
							ret = kstrtoull(prvi, 0, &drugi);
							broj_ceo=drugi;							
							break;
						case 4:
							prvi[0]=buff[i-4]; 
							prvi[1]=buff[i-3];
							prvi[2]=buff[i-2];
							prvi[3]=buff[i-1];
							prvi[4] = '\0';
							ret = kstrtoull(prvi, 0, &drugi);
							broj_ceo=drugi;
							break;
					}
					
					if(broj_ceo > 4095){
						printk(KERN_WARNING "VGA_write: Neki od unetih brojeva prelazi 4095, upis se stopira.\n");
						goto skoro_zavrsetak;
					}

					uneti_brojevi_a_reverse[u]=broj_ceo;
					u = u + 1;
					koliko_unetih_brojeva_a = u;
					break;
				}
			}
		}
			
        if(i == 0){
				break;
		}
		i = i - 1; 
		
	    }
	  
	  
	    for(i=0 ; i<u; i=i+1){ 
		    uneti_brojevi_a[i] = uneti_brojevi_a_reverse[koliko_unetih_brojeva_a-i-1];
	    }
	  
	    //*************KRAJ FUNKCIJE ZA RACUNANJE BROJA CIFARA I KOJE SU TO CIFRE****************************
	  
	    //************PROVERA DA LI JE UNESENO DOVOLJNO ILI PREKO BROJEVA OD ONOGA STO TREBA******************
	  
	    if(((koliko_unetih_brojeva_a%prava_dim_n) != 0) || ((koliko_unetih_brojeva_a%prava_dim_m) != 0)){  
		    //ovaj if kaze da ako je broj uetih brojeva mora biti broj kolone*broj reda
		    goto skoro_zavrsetak;
	    }
	  
	    //************ KRAJ PROVERE DA LI JE UNESENO DOVOLJNO ILI PREKO BROJEVA OD ONOGA STO TREBA******************
	  
	    if((prava_dim_n == ioread32(vm->base_addr + 8)) && ((prava_dim_m==ioread32(vm->base_addr + 12)))){ // proverava da li se dimenzije poklapaju sa onim iz matmul
	    
	        for(i = 0 ; i < koliko_unetih_brojeva_a ; i = i+1){
		        upis = uneti_brojevi_a[i];
		        iowrite32(upis, va->base_addr + 4*i);
		        printk(KERN_WARNING "VGA_write: upisano %u u va->base_addr + 4*%u\n",upis, i);  
	        }

			//*******prebacujemo jednodimenzionalni niz u dvodimenzionalni niz tj. matricu*****************************************
			u = 0;
			for( i = 0 ; i < prava_dim_n ; i = i + 1){
					for(k = 0 ; k < prava_dim_m ; k = k + 1){
						uneti_brojevi_a_matrix[i][k] = uneti_brojevi_a[u];
						u = u + 1;
						printk(KERN_WARNING "VGA_write: uneti_brojevi_a_matrix[%u][%u] = %u\n",i, k, uneti_brojevi_a_matrix[i][k]);
					}
			}

			//*************kraj prebacivanja u matricu***********************************************************************
	        printk(KERN_WARNING "VGA_write: poklapaju se n,m i p iz matmul i ovi uneti");
	        goto zavrsetak; 
	    } else{
	        printk(KERN_WARNING "VGA_write: ne poklapaju se n,m i/ili p iz matmul i ovi uneti");
	        goto skoro_zavrsetak; 
	    }
	  
    } //ovo je kraj if(minor == 0) tj. ako upisujemo u bram_a


//**********************************************************************************************************************************88
    skoro_zavrsetak:
    printk(KERN_WARNING "VGA_write: pogresan unos");
    zavrsetak: 
    return length;
}

//################################################### INIT ###################################################3
static int __init matmul_init(void)
{
    int ret = 0;
    int_cnt = 0;

  printk(KERN_INFO "matmul_init: Initialize Module \"%s\"\n", DEVICE_NAME);
  ret = alloc_chrdev_region(&my_dev_id, 0, 4, "matmul_region");
  if (ret)
  {
    printk(KERN_ALERT "<1>Failed CHRDEV!.\n");
    return -1;
  }
  printk(KERN_INFO "Succ CHRDEV!.\n");
  my_class = class_create(THIS_MODULE, "matmul_drv");
  if (my_class == NULL)
  {
    printk(KERN_ALERT "<1>Failed class create!.\n");
    goto fail_0;
  }
  printk(KERN_INFO "Succ class chardev1 create!.\n");
  
   printk(KERN_INFO "created nod %d\n", 1);
    
    my_device = device_create(my_class, NULL, MKDEV(MAJOR(my_dev_id), 0), NULL, "bram_a");
  
  if (my_device == NULL)
  {
    goto fail_1;
  }
  printk(KERN_INFO "created nod %d\n", 2);
   my_device = device_create(my_class, NULL, MKDEV(MAJOR(my_dev_id),1), NULL, "bram_b");
   
  if (my_device == NULL)
  {
    goto fail_1;
  }
  
    printk(KERN_INFO "created nod %d\n", 3);
   my_device = device_create(my_class, NULL, MKDEV(MAJOR(my_dev_id),2), NULL, "bram_c");
   
  if (my_device == NULL)
  {
    goto fail_1;
  }

    printk(KERN_INFO "created nod %d\n", 4);
   my_device = device_create(my_class, NULL, MKDEV(MAJOR(my_dev_id),3), NULL, "matmul");
   
  if (my_device == NULL)
  {
    goto fail_1;
  }
  printk(KERN_INFO "Device created.\n");

  my_cdev = cdev_alloc();	
  my_cdev->ops = &my_fops;
  my_cdev->owner = THIS_MODULE;
  ret = cdev_add(my_cdev, my_dev_id, 4);
  if (ret)
  {
    printk(KERN_ERR "matmul_init: Failed to add cdev\n");
    goto fail_2;
  }
  printk(KERN_INFO "matmul device init.\n");

  return platform_driver_register(&matmul_driver);

 fail_2:
  device_destroy(my_class, my_dev_id );
 fail_1:
  class_destroy(my_class);
 fail_0:
  unregister_chrdev_region(my_dev_id, 1);
  return -1;
}

//################################################# EXIT  ##################################################
static void __exit matmul_exit(void)
{
  platform_driver_unregister(&matmul_driver);
  cdev_del(my_cdev);
  device_destroy(my_class, MKDEV(MAJOR(my_dev_id),0));
  device_destroy(my_class, MKDEV(MAJOR(my_dev_id),1));
  device_destroy(my_class, MKDEV(MAJOR(my_dev_id),2));
  device_destroy(my_class, MKDEV(MAJOR(my_dev_id),3));
  class_destroy(my_class);
  unregister_chrdev_region(my_dev_id, 4);
  printk(KERN_INFO "matmul driver closed.\n");
}

//#################################################################################################
module_init(matmul_init);
module_exit(matmul_exit);
