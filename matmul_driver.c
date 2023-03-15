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



MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Drajver za mnozenje matrica");
#define DEVICE_NAME "matrix_multiplier" //bilo vga
#define DRIVER_NAME "matrix_multiplier_driver"//bilo vga_driver
#define BUFF_SIZE 250


//*******************PROTOTIPI FUNKCIJA************************************

static int matrix_multiplier_probe(struct platform_device *pdev);
static int matrix_multiplier_open(struct inode *i, struct file *f);
static int matrix_multiplier_close(struct inode *i, struct file *f);
static ssize_t matrix_multiplier_read(struct file *f, char __user *buf, size_t len, loff_t *off);
static ssize_t matrix_multiplier_write(struct file *f, const char __user *buf, size_t length, loff_t *off);
static int __init matrix_multiplier_init(void);
static void __exit matrix_multiplier_exit(void);
static int matrix_multiplier_remove(struct platform_device *pdev);

//*********************GLOBALNE PROMENLJIVE*************************************
struct matrix_multiplier_info {
  unsigned long mem_start;
  unsigned long mem_end;
  void __iomem *base_addr;
};
static struct cdev *my_cdev;
static dev_t my_dev_id;
static struct class *my_class;
static struct device *my_device;
static int int_cnt;
static struct matrix_multiplier_info *va = NULL;
static struct matrix_multiplier_info *vb = NULL;
static struct matrix_multiplier_info *vc = NULL;
static struct matrix_multiplier_info *vm = NULL;

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

//unsigned int rezultat_mnozenja[8][8];

//*********************OPERACIJE NAD NODE FAJLOVIMA*************************************
static struct file_operations my_fops =
  {
    .owner = THIS_MODULE,
    .open = matrix_multiplier_open,
    .release = matrix_multiplier_close,
    .read = matrix_multiplier_read,
    .write = matrix_multiplier_write
  };

//*********************POVEZUJE INFORMACIJE SA STABLOM UREĐAJA*************************************  
static struct of_device_id matrix_multiplier_of_match[] = {
  { .compatible = "xlnx,axi-bram-ctrl-A", },
  { .compatible = "xlnx,axi-bram-ctrl-B", },
  { .compatible = "xlnx,axi-bram-ctrl-C", },
  { .compatible = "xlnx,matrix-multiplier", },
  { /* end of list */ },
};

static struct platform_driver matrix_multiplier_driver = {
  .driver = {
    .name = DRIVER_NAME,
    .owner = THIS_MODULE,
    .of_match_table	= matrix_multiplier_of_match,
  },
  .probe		= matrix_multiplier_probe,
  .remove	= matrix_multiplier_remove,
};

MODULE_DEVICE_TABLE(of, matrix_multiplier_of_match);


//***************************************************************************

//*********************PROBE AND REMOVE*************************************  

static int matrix_multiplier_probe(struct platform_device *pdev)
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
    printk(KERN_ALERT "matrix_multiplier_probe: Failed to get reg resource\n");
    return -ENODEV;
  }
  // Get memory for structure vga_info
  va = (struct matrix_multiplier_info *) kmalloc(sizeof(struct matrix_multiplier_info), GFP_KERNEL);
  if (!va) {
    printk(KERN_ALERT "matrix_multiplier_probe: Could not allocate timer device\n");
    return -ENOMEM;
  }
  // Put phisical adresses in timer_info structure
  va->mem_start = r_mem->start;
  va->mem_end = r_mem->end;
  
  printk(KERN_INFO "Start address:%x \t end address:%x", r_mem->start, r_mem->end);
    
  // Reserve that memory space for this driver
  if (!request_mem_region(va->mem_start,va->mem_end - va->mem_start + 1, DRIVER_NAME))
  {
    printk(KERN_ALERT "matrix_multiplier_probe: Could not lock memory region at %p\n",(void *)va->mem_start);
    rc = -EBUSY;
    goto error1;
  }    
  // Remap phisical to virtual adresses

  va->base_addr = ioremap(va->mem_start, va->mem_end - va->mem_start + 1);
  if (!va->base_addr) {
    printk(KERN_ALERT "matrix_multiplier_probe: Could not allocate memory\n");
    rc = -EIO;
    goto error2;
  }

  printk(KERN_NOTICE "matrix_multiplier_probe: BRAM_CTRL_A uredjaj ubacen\n");
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
    printk(KERN_ALERT "matrix_multiplier_probe: Failed to get reg resource\n");
    return -ENODEV;
  }
  // Get memory for structure vga_info
  vb = (struct matrix_multiplier_info *) kmalloc(sizeof(struct matrix_multiplier_info), GFP_KERNEL);
  if (!vb) {
    printk(KERN_ALERT "matrix_multiplier_probe: Could not allocate timer device\n");
    return -ENOMEM;
  }
  // Put phisical adresses in timer_info structure
  vb->mem_start = r_mem2->start;
  vb->mem_end = r_mem2->end;
  
  printk(KERN_INFO "Start address:%x \t end address:%x", r_mem2->start, r_mem2->end);
    
  // Reserve that memory space for this driver
  if (!request_mem_region(vb->mem_start,vb->mem_end - vb->mem_start + 1, DRIVER_NAME))
  {
    printk(KERN_ALERT "matrix_multiplier_probe: Could not lock memory region at %p\n",(void *)vb->mem_start);
    rc2 = -EBUSY;
    goto greska1;
  }    
  // Remap phisical to virtual adresses

  vb->base_addr = ioremap(vb->mem_start, vb->mem_end - vb->mem_start + 1);
  if (!vb->base_addr) {
    printk(KERN_ALERT "matrix_multiplier_probe: Could not allocate memory\n");
    rc2 = -EIO;
    goto greska2;
  }

  printk(KERN_NOTICE "matrix_multiplier_probe: BRAM_CTRL_B uredjaj ubacen\n");
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
    printk(KERN_ALERT "matrix_multiplier_probe: Failed to get reg resource\n");
    return -ENODEV;
  }
  // Get memory for structure vga_info
  vc = (struct matrix_multiplier_info *) kmalloc(sizeof(struct matrix_multiplier_info), GFP_KERNEL);
  if (!vc) {
    printk(KERN_ALERT "matrix_multiplier_probe: Could not allocate timer device\n");
    return -ENOMEM;
  }
  // Put phisical adresses in timer_info structure
  vc->mem_start = r_mem3->start;
  vc->mem_end = r_mem3->end;
  
  printk(KERN_INFO "Start address:%x \t end address:%x", r_mem3->start, r_mem3->end);
    
  // Reserve that memory space for this driver
  if (!request_mem_region(vc->mem_start,vc->mem_end - vc->mem_start + 1, DRIVER_NAME))
  {
    printk(KERN_ALERT "matrix_multiplier_probe: Could not lock memory region at %p\n",(void *)vc->mem_start);
    rc3 = -EBUSY;
    goto greska3;
  }    
  // Remap phisical to virtual adresses

  vc->base_addr = ioremap(vc->mem_start, vc->mem_end - vc->mem_start + 1);
  if (!vc->base_addr) {
    printk(KERN_ALERT "matrix_multiplier_probe: Could not allocate memory\n");
    rc3 = -EIO;
    goto greska4;
  }

  printk(KERN_NOTICE "matrix_multiplier_probe: BRAM_CTRL_C uredjaj ubacen\n");
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
    printk(KERN_ALERT "matrix_multiplier_probe: Failed to get reg resource\n");
    return -ENODEV;
  }
  // Get memory for structure vga_info
  vm = (struct matrix_multiplier_info *) kmalloc(sizeof(struct matrix_multiplier_info), GFP_KERNEL);
  if (!vm) {
    printk(KERN_ALERT "matrix_multiplier_probe: Could not allocate timer device\n");
    return -ENOMEM;
  }
  // Put phisical adresses in timer_info structure
  vm->mem_start = r_mem4->start;
  vm->mem_end = r_mem4->end;
  
  printk(KERN_INFO "Start address:%x \t end address:%x", r_mem4->start, r_mem4->end);
    
  // Reserve that memory space for this driver
  if (!request_mem_region(vm->mem_start,vm->mem_end - vm->mem_start + 1, DRIVER_NAME))
  {
    printk(KERN_ALERT "matrix_multiplier_probe: Could not lock memory region at %p\n",(void *)vm->mem_start);
    rc4 = -EBUSY;
    goto greska5;
  }    
  // Remap phisical to virtual adresses

  vm->base_addr = ioremap(vm->mem_start, vm->mem_end - vm->mem_start + 1);
  if (!vm->base_addr) {
    printk(KERN_ALERT "matrix_multiplier_probe: Could not allocate memory\n");
    rc4 = -EIO;
    goto greska6;
  }

  printk(KERN_NOTICE "matrix_multiplier_probe: MATRIX_MULTIPLIER uredjaj ubacen\n");
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

static int matrix_multiplier_remove(struct platform_device *pdev)
{
 
  if((request_mem_region(va->mem_start,va->mem_end - va->mem_start + 1, DRIVER_NAME)== NULL)){
	  
  printk(KERN_INFO "matrix_multiplier_remove: va remove in process");
  iounmap(va->base_addr);
  release_mem_region(va->mem_start, va->mem_end - va->mem_start + 1);
  printk(KERN_INFO "matrix_multiplier_remove: matrix_multiplier_driver removed va");
  goto kraj1;
  
  }
  
  else if ((request_mem_region(vb->mem_start,vb->mem_end - vb->mem_start + 1, DRIVER_NAME)== NULL)){
	  
  printk(KERN_INFO "matrix_multiplier_remove: vb remove in process");
  iounmap(vb->base_addr);
  release_mem_region(va->mem_start, va->mem_end - va->mem_start + 1);
  release_mem_region(vb->mem_start, vb->mem_end - vb->mem_start + 1);
  printk(KERN_INFO "matrix_multiplier_remove: matrix_multiplier_driver removed vb");
  goto kraj1;
  
  
  }
   
    else if ((request_mem_region(vc->mem_start,vc->mem_end - vc->mem_start + 1, DRIVER_NAME)== NULL)){
		
  printk(KERN_INFO "matrix_multiplier_remove: vc remove in process");
  iounmap(vc->base_addr);
  release_mem_region(va->mem_start, va->mem_end - va->mem_start + 1);
  release_mem_region(vb->mem_start, vb->mem_end - vb->mem_start + 1);
  release_mem_region(vc->mem_start, vc->mem_end - vc->mem_start + 1);
  printk(KERN_INFO "matrix_multiplier_remove: matrix_multiplier_driver removed vc");
  goto kraj1;
  
  }
  
   else {
	   
  printk(KERN_INFO "matrix_multiplier_remove: vm remove in process");
  iounmap(vm->base_addr);
  release_mem_region(va->mem_start, va->mem_end - va->mem_start + 1);
  release_mem_region(vb->mem_start, vb->mem_end - vb->mem_start + 1);
  release_mem_region(vc->mem_start, vc->mem_end - vc->mem_start + 1);
  release_mem_region(vm->mem_start, vm->mem_end - vm->mem_start + 1);
  printk(KERN_INFO "matrix_multiplier_remove: matrix_multiplier_driver removed vm");
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

//***************************************************
// IMPLEMENTACIJA FAJL OPERACIJA
//***************************************************

static int matrix_multiplier_open(struct inode *i, struct file *f)
{
  printk(KERN_INFO "matrix_multiplier opened\n");
  return 0;
}
static int matrix_multiplier_close(struct inode *i, struct file *f)
{
  printk(KERN_INFO "matrix_multiplier closed\n");
  return 0;
}
static ssize_t matrix_multiplier_read(struct file *f, char __user *buf, size_t len, loff_t *off)
{
  unsigned int ready_r,start_r,n_r,m_r,p_r;
  unsigned int vrednost;
  unsigned int petlja_brojac = 0;
  unsigned int broj_kolona, broj_redova;
  int minor;
  
  printk("matrix_multiplier read\n");
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
  if(minor == 1){ //citamo iz bram_b
	  
	  broj_kolona = ioread32(vm->base_addr + 16); // pročitaj p iz matmul
	  broj_redova = ioread32(vm->base_addr + 12);  // pročitaj m iz matmul
	  
	  for (petlja_brojac = 0;petlja_brojac < (broj_kolona*broj_redova) ; petlja_brojac = petlja_brojac + 1){
		  
		  vrednost = ioread32(vb->base_addr + 4*petlja_brojac);
		  //printk("Broj kolona je %u a broj redova je %u ",broj_kolona, broj_redova);
		  printk(KERN_CONT "%u",vrednost);
		  
		  if((petlja_brojac+1)%broj_kolona == 0){
			  
			 printk(KERN_CONT ";");
			 
		  }
		  else {
			  printk(KERN_CONT ",");
		  }
	  }
	  
	  goto procitano;
  }
   if(minor == 2){ //citamo iz bram_c
	  
	  broj_kolona = ioread32(vm->base_addr + 16); // pročitaj p iz matmul
	  broj_redova = ioread32(vm->base_addr + 8);  // pročitaj n iz matmul
	  
	  for (petlja_brojac = 0;petlja_brojac < (broj_kolona*broj_redova) ; petlja_brojac = petlja_brojac + 1){
		  
		  vrednost = ioread32(vc->base_addr + 4*petlja_brojac);
		  //printk("Broj kolona je %u a broj redova je %u ",broj_kolona, broj_redova);
		  printk(KERN_CONT "%u",vrednost);
		  
		  if((petlja_brojac+1)%broj_kolona == 0){
			  
			 printk(KERN_CONT ";");
			 
		  }
		  else {
			  printk(KERN_CONT ",");
		  }
	  }
	  
	  goto procitano;
  }
  if(minor == 3){ //citamo iz matmul
	  
	  ready_r = ioread32(vm->base_addr);
	  start_r = ioread32(vm->base_addr+4);
	  n_r = ioread32(vm->base_addr+8);
	  m_r = ioread32(vm->base_addr+12);
	  p_r = ioread32(vm->base_addr+16);
	  printk("ready=%u;start=%u;n=%u;m=%u;p=%u\n",ready_r, start_r, n_r, m_r, p_r);
	  goto procitano;
	  
  }
  procitano:
  return 0;
}
static ssize_t matrix_multiplier_write(struct file *f, const char __user *buf, size_t length, loff_t *off)
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
  
    minor = MINOR(f->f_inode->i_rdev);
    ret = copy_from_user(buff, buf, length); 
  
  if(ret){
    printk("copy from user failed \n");
    return -EFAULT;
  }  
    buff[length] = '\0';
	
   
	
  if(minor == 3)
 { //ako upisujemo u matumul
	  
	  if(strncmp(buff, n, 2)==0){  // ispituje da li je uneto pod navodnicima n
		  
		   buff_pomocni[0] = buff[2]; // ako jeste onda napravi novi niz u koji se smesta broj koji je unet kao n
	       buff_pomocni[1] = '\0';  // dodaj mu terminator
		  
		  if((strlen(buff)>4) || (strcmp(buff_pomocni,osmica) == 0) || (strcmp(buff_pomocni,devetka) == 0)){
			  
			  printk(KERN_WARNING "VGA_write: dimenzija n mora biti manja ili jednaka 7\n");
			  goto skoro_zavrsetak;
			  
		  }
		  prvi[0]=buff[2]; 
		  prvi[1] = '\0';
		  ret = kstrtoull(prvi, 0, &drugi);
		  drugi_manji=drugi;
		  iowrite32(drugi_manji, vm->base_addr + 8);
		  printk(KERN_WARNING "VGA_write: uspesno upisano %lu u vm->base_addr + 8\n", drugi_manji);
		  goto zavrsetak;
	  }
	  
	  else if(strncmp(buff, m, 2)==0){
		  
		   buff_pomocni[0] = buff[2];
	       buff_pomocni[1] = '\0';
		  
		  if((strlen(buff)>4) || (strcmp(buff_pomocni,osmica) == 0) || (strcmp(buff_pomocni,devetka) == 0)){
			  
			  printk(KERN_WARNING "VGA_write: dimenzija m mora biti manja ili jednaka 7\n");
			  goto skoro_zavrsetak;
			  
		  }
		  prvi[0]=buff[2]; 
		  prvi[1] = '\0';
		  ret = kstrtoull(prvi, 0, &drugi);
		  drugi_manji=drugi;
		  iowrite32(drugi_manji, vm->base_addr + 12);
		  printk(KERN_WARNING "VGA_write: uspesno upisano %lu u vm->base_addr + 12\n", drugi_manji);
		  goto zavrsetak;
		  //}
		  //else{goto skoro_zavrsetak;}
	  }
	  
	  else if(strncmp(buff, p, 2)==0){
		  
		  buff_pomocni[0] = buff[2];
	      buff_pomocni[1] = '\0';
		  
		  if((strlen(buff)>4) || (strcmp(buff_pomocni,osmica) == 0) || (strcmp(buff_pomocni,devetka) == 0)){
			  
			  printk(KERN_WARNING "VGA_write: dimenzija p mora biti manja ili jednaka 7\n");
			  goto skoro_zavrsetak;
			  
		  }
		  prvi[0]=buff[2]; 
		  prvi[1] = '\0';
		  ret = kstrtoull(prvi, 0, &drugi);
		  drugi_manji=drugi;
		  iowrite32(drugi_manji, vm->base_addr + 16);
		  printk(KERN_WARNING "VGA_write: uspesno upisano %lu u vm->base_addr + 16\n", drugi_manji);
		  goto zavrsetak;
	  }
	  
	  else if(strncmp(buff, start, 6)==0){
		  
		  buff_pomocni[0] = buff[6];
	      buff_pomocni[1] = '\0';
		  
		    buff_pomocni2[0] = buff[6];
		    buff_pomocni2[1] = buff[7];
			buff_pomocni2[2] = buff[8];
			buff_pomocni2[3] = buff[9];
			buff_pomocni2[4] = buff[10];
			buff_pomocni2[5] = buff[11];
			buff_pomocni2[6] = buff[12];
			buff_pomocni2[7] = '\0';
		  
		  if((strcmp(buff_pomocni,jedan) == 0) || (strcmp(buff_pomocni,nula) == 0) || (strcmp(buff_pomocni2,trig)==0)){
			  
			  if(strcmp(buff_pomocni,jedan)==0){
				  
				  iowrite32(1, vm->base_addr + 4);
				  printk(KERN_WARNING "VGA_write: uspesno upisan 1 u vm->base_addr + 4\n");
				  goto zavrsetak;
			  }
			  
			  if(strcmp(buff_pomocni,nula)==0){
				  
				  if((ioread32(vm->base_addr + 4))==1){
					  
				  iowrite32(0, vm->base_addr + 4);
				  printk(KERN_WARNING "VGA_write: uspesno upisana 0 posle jedinice u vm->base_addr + 4\n");
				  printk(KERN_WARNING "VGA_write: zapocinje se mnozenje\n");
				  
				  broj_redova_a = ioread32(vm->base_addr + 8);  // pročitaj n iz matmul
				  broj_kolona_b = ioread32(vm->base_addr + 16);  // pročitaj p iz matmul
				  broj_kolona_a = ioread32(vm->base_addr + 12);  // pročitaj m iz matmul
				  
			  for( i = 0 ; i < broj_redova_a ; i = i + 1){
				  
				  for( k = 0 ; k < broj_kolona_b ; k = k + 1){
					  
					  uneti_brojevi_c_matrix[i][k] = 0;
					  
					  for( q = 0 ; q < broj_kolona_a ; q = q + 1){
						  
						  uneti_brojevi_c_matrix[i][k] = uneti_brojevi_c_matrix[i][k] + uneti_brojevi_a_matrix[i][q] * uneti_brojevi_b_matrix[q][k];
						  
					  } // kraj for petlje sa q	
					  
				  } //kraj for sa k
				  
				  
			  } // kraj for sa i
				  
				  
				  
				  }
				  
				  else{
					  iowrite32(0, vm->base_addr + 4);
					  printk(KERN_WARNING "VGA_write: uspesno upisana 0 u vm->base_addr + 4\n");
				  }
				  goto zavrsetak;
			  }
			  
			  if(strcmp(buff_pomocni2,trig)==0){
				  
				  iowrite32(1, vm->base_addr + 4);
				  printk(KERN_WARNING "VGA_write: uspesno upisan 1 u vm->base_addr + 4\n");
				  iowrite32(0, vm->base_addr + 4);
				  printk(KERN_WARNING "VGA_write: uspesno upisana 0 u vm->base_addr + 4\n");
				  
				  broj_redova_a = ioread32(vm->base_addr + 8);  // pročitaj n iz matmul
				  broj_kolona_b = ioread32(vm->base_addr + 16);  // pročitaj p iz matmul
				  broj_kolona_a = ioread32(vm->base_addr + 12);  // pročitaj m iz matmul
				  
			  for( i = 0 ; i < broj_redova_a ; i = i + 1){
				  
				  for( k = 0 ; k < broj_kolona_b ; k = k + 1){
					  
					  uneti_brojevi_c_matrix[i][k] = 0;
					  
					  for( q = 0 ; q < broj_kolona_a ; q = q + 1){
						  
						  uneti_brojevi_c_matrix[i][k] = uneti_brojevi_c_matrix[i][k] + uneti_brojevi_a_matrix[i][q] * uneti_brojevi_b_matrix[q][k];
						  
					  } // kraj for petlje sa q	
					  
				  } //kraj for sa k
				  
				  
			  } // kraj for sa i
			  //***************IZRACUNATO MNOZENJE IZNAD***********************************************************
			  /*
			  for( i = 0 ; i < broj_redova_a ; i = i + 1){	
				
					for(k = 0 ; k < broj_kolona_b ; k = k + 1){
						
						printk(KERN_WARNING "VGA_write: Rezultat mnozenja : uneti_brojevi_c_matrix[%u][%u] = %u\n",i, k, uneti_brojevi_c_matrix[i][k]);
						
					}
			}
			  
			  */
			  //*****************************************************************************************
			  
				  goto zavrsetak;
			  }
			  
		  }
		  else{
			  
			  printk(KERN_WARNING "VGA_write: start moze imati vrednosti 0,1 ili trigger\n");
			  goto skoro_zavrsetak; 
			  
		  }
		  
	  }
	  
  }
  //----------------------------------------------------------------------
  
  if(minor == 0){ //ako upisujemo u bram_a
	  
	  brojac = 0;
	  prava_dim_m = 0;
	  dimenzija_m = strchr(buff, ';'); // trazi prvo pojavljivanje znaka ; u stringu buff i taj broj smesta u dimenzija_p
	  //printk(KERN_WARNING "VGA_write: dimenzija_p = %u\n",dimenzija_p); // ovo daje samo adresu jer je pokazivac
	  
	  razlika_adresa = dimenzija_m - buff;
	  
	  //printk(KERN_WARNING "VGA_write: razlika_adresa = %d\n",razlika_adresa); //ovo je razlika adresa za potrebe debugovanja
	  
	  for( i=0; i<razlika_adresa; i=i+1 ){
		  
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
	  //printk(KERN_WARNING "VGA_write: Duzina buff = %d\n",length);
	   
	  
	  for(i=0 ; i<length; i=i+1){  //sluzi da smesti pozicije zareza
		  
		  if((buff[i]==',') || (buff[i]==';')){
			  pozicija_zareza[i] = i;
			  
		  }
		  else{
			  pozicija_zareza[i] = 0;
		  }
		  
	  }
	  
	  
	  for(i=0 ; i<length; i=i+1){ // sluzi da ispise pozicije zareza za potrebe debugovanja
	  
		//printk(KERN_WARNING "VGA_write: pozicija_zareza[%u] = %u\n",i, pozicija_zareza[i]);
	  
	  }
	  //**********FUNKCIJA ZA RACUNANJE BROJA CIFARA I KOJE SU TO CIFRE***********************
	  i = length - 1;
	  u = 0; // brojac za niz uneti_brojevi_b,a
	  while(i>=0){
		  //printk(KERN_WARNING "VGA_write: tek poceo i = %d\n",i);
		if(pozicija_zareza[i]!=0){
			//printk(KERN_WARNING "VGA_write: jeste i razlicito od 0");
			
			veci_indeks = pozicija_zareza[i];
			//printk(KERN_WARNING "VGA_write: veci indeks je = %u", veci_indeks);
			
			for(k = i-1; k >= 0; k = k - 1){
				//printk(KERN_WARNING "VGA_write: usao u petlju sa k = %d", k);
				if((pozicija_zareza[k]!=0) || (k==0)){
					manji_indeks = pozicija_zareza[k];
					//printk(KERN_WARNING "VGA_write: manji indeks je = %u", manji_indeks);
					if(k==0){
						koliko_cifara = veci_indeks-manji_indeks;
					}
					else{
					koliko_cifara = veci_indeks-manji_indeks-1;
					}
					//printk(KERN_WARNING "VGA_write: koliko cifara je = %u", koliko_cifara);
					
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
					//printk(KERN_WARNING "VGA_write: broj_ceo = %lu\n",broj_ceo);
					if(broj_ceo > 4095){
						
						printk(KERN_WARNING "VGA_write: Neki od unetih brojeva prelazi 4095, upis se stopira.\n");
						goto skoro_zavrsetak;
						
					}
					uneti_brojevi_a_reverse[u]=broj_ceo;
					u = u + 1;
					koliko_unetih_brojeva_a = u;
					break;
					
				}
				//break;
			}
		
		}
			if(i == 0){
				break;
			}
		 i = i - 1; 
		 //printk(KERN_WARNING "VGA_write: tek zavrsavam i = %d\n",i);
	  }
	  
	  
	  for(i=0 ; i<u; i=i+1){ // sluzi da napravi non reverse raspored i ispise unos_brojeva_b za potrebe debugovanja
		
		uneti_brojevi_a[i] = uneti_brojevi_a_reverse[koliko_unetih_brojeva_a-i-1];
	    //printk(KERN_WARNING "VGA_write: unos[%u] = %u",i, uneti_brojevi_a[i]);
	  
	  }
	  
	  //*************KRAJ FUNKCIJE ZA RACUNANJE BROJA CIFARA I KOJE SU TO CIFRE****************************
	  
	  //************PROVERA DA LI JE UNESENO DOVOLJNO ILI PREKO BROJEVA OD ONOGA STO TREBA******************
	  
	  
	  if(((koliko_unetih_brojeva_a%prava_dim_n) != 0) || ((koliko_unetih_brojeva_a%prava_dim_m) != 0)){
		  
		  //ovaj if kaze da ako je broj uetih brojeva mora biti broj kolone*broj reda
		  //printk(KERN_WARNING "VGA_write: Uneo si ili vise ili manje brojeva nego sto treba.\n");
		  goto skoro_zavrsetak;
	  }
	  
	   //************ KRAJ PROVERE DA LI JE UNESENO DOVOLJNO ILI PREKO BROJEVA OD ONOGA STO TREBA******************
	   
	  /*
	  for(i = 2 ; i <= prava_dim_m ; i = i+1){ // proverava da li su svi redovi i kolone iste tj. da li se svi znakovi ; nalaze na istim udaljenostima jedni od drugih
		  	  
	  if(buff[prava_dim_m + prava_dim_p]!=buff[i*prava_dim_p + i*prava_dim_m + i-1]){
		  
		  printk(KERN_WARNING "VGA_write: los unos tj. pogresan\n");
		  goto skoro_zavrsetak;
		  
		}
	  }
	  */
	  
	  if((prava_dim_n == ioread32(vm->base_addr + 8)) && ((prava_dim_m==ioread32(vm->base_addr + 12)))) // proverava da li se dimenzije poklapaju sa onim iz matmul
	  {
		  
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
	  }
	  else{
	    printk(KERN_WARNING "VGA_write: ne poklapaju se n,m i/ili p iz matmul i ovi uneti");
	    goto skoro_zavrsetak; 
	  }
	  
  //----------------------------------------------------------------------------------------------
  
  } //ovo je kraj if(minor == 0) tj. ako upisujemo u bram_a
  
  //----------------------------------------------------------------------------------------------
  
  if(minor == 1){ //ako upisujemo u bram_b
	  
	  brojac = 0;
	  prava_dim_p = 0;
	  dimenzija_p = strchr(buff, ';'); // trazi prvo pojavljivanje znaka ; u stringu buff i taj broj smesta u dimenzija_p
	  //printk(KERN_WARNING "VGA_write: dimenzija_p = %u\n",dimenzija_p); // ovo daje samo adresu jer je pokazivac
	  
	  razlika_adresa = dimenzija_p - buff;
	  
	  //printk(KERN_WARNING "VGA_write: razlika_adresa = %d\n",razlika_adresa); //ovo je razlika adresa za potrebe debugovanja
	  
	  for( i=0; i<razlika_adresa; i=i+1 ){
		  
		  if(buff[i]==','){
			  prava_dim_p = prava_dim_p + 1;
		  }
		  
	  }
	  prava_dim_p = prava_dim_p + 1; // treba da bi se dobila tacna vrednost jer je na kraju reda uvek ; a ne ,
	  printk(KERN_WARNING "VGA_write: dimenzija_p = %d\n",prava_dim_p); //ovo je dimenzija dobijena iz unosa (ne iz matmula)
	  
	  for( i=0; i<length; i=i+1 ){
		  
		  if(buff[i]==';'){
			  prava_dim_m = prava_dim_m + 1;
		  }
		  
	  }
	  printk(KERN_WARNING "VGA_write: dimenzija_m = %u\n",prava_dim_m); //ovo je dimenzija dobijena iz unosa (ne iz matmula)
	  //printk(KERN_WARNING "VGA_write: Duzina buff = %d\n",length);
	  
	  
	  for(i=0 ; i<length; i=i+1){  //sluzi da smesti pozicije zareza
		  
		  if((buff[i]==',') || (buff[i]==';')){
			  pozicija_zareza[i] = i;
			  
		  }
		  else{
			  pozicija_zareza[i] = 0;
		  }
		  
	  }
	  
	  
	  for(i=0 ; i<length; i=i+1){ // sluzi da ispise pozicije zareza za potrebe debugovanja
	  
		//printk(KERN_WARNING "VGA_write: pozicija_zareza[%u] = %u\n",i, pozicija_zareza[i]);
	  
	  }
	  //**********FUNKCIJA ZA RACUNANJE BROJA CIFARA I KOJE SU TO CIFRE***********************
	  i = length - 1;
	  u = 0; // brojac za niz uneti_brojevi_b
	  while(i>=0){
		  //printk(KERN_WARNING "VGA_write: tek poceo i = %d\n",i);
		if(pozicija_zareza[i]!=0){
			//printk(KERN_WARNING "VGA_write: jeste i razlicito od 0");
			
			veci_indeks = pozicija_zareza[i];
			//printk(KERN_WARNING "VGA_write: veci indeks je = %u", veci_indeks);
			
			for(k = i-1; k >= 0; k = k - 1){
				//printk(KERN_WARNING "VGA_write: usao u petlju sa k = %d", k);
				if((pozicija_zareza[k]!=0) || (k==0)){
					manji_indeks = pozicija_zareza[k];
					//printk(KERN_WARNING "VGA_write: manji indeks je = %u", manji_indeks);
					if(k==0){
						koliko_cifara = veci_indeks-manji_indeks;
					}
					else{
					koliko_cifara = veci_indeks-manji_indeks-1;
					}
					//printk(KERN_WARNING "VGA_write: koliko cifara je = %u", koliko_cifara);
					
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
					//printk(KERN_WARNING "VGA_write: broj_ceo = %lu\n",broj_ceo);
					if(broj_ceo > 4095){
						
						printk(KERN_WARNING "VGA_write: Neki od unetih brojeva prelazi 4095, upis se stopira.\n");
						goto skoro_zavrsetak;
						
					}
					uneti_brojevi_b_reverse[u]=broj_ceo;
					u = u + 1;
					koliko_unetih_brojeva_b = u;
					break;
					
				}
				//break;
			}
		
		}
			if(i == 0){
				break;
			}
		 i = i - 1; 
		 //printk(KERN_WARNING "VGA_write: tek zavrsavam i = %d\n",i);
	  }
	  
	  
	  for(i=0 ; i<u; i=i+1){ // sluzi da napravi non reverse raspored i ispise unos_brojeva_b za potrebe debugovanja
		
		uneti_brojevi_b[i] = uneti_brojevi_b_reverse[koliko_unetih_brojeva_b-i-1];
	    //printk(KERN_WARNING "VGA_write: unos[%u] = %u",i, uneti_brojevi_b[i]);
	  
	  }
	  
	  //*************KRAJ FUNKCIJE ZA RACUNANJE BROJA CIFARA I KOJE SU TO CIFRE****************************
	  
	  //************PROVERA DA LI JE UNESENO DOVOLJNO ILI PREKO BROJEVA OD ONOGA STO TREBA******************
	  
	  
	  if(((koliko_unetih_brojeva_b%prava_dim_p) != 0) || ((koliko_unetih_brojeva_b%prava_dim_m) != 0)){
		  
		  //ovaj if kaze da ako je broj uetih brojeva mora biti broj kolone*broj reda
		  //printk(KERN_WARNING "VGA_write: Uneo si ili vise ili manje brojeva nego sto treba.\n");
		  goto skoro_zavrsetak;
	  }
	  
	   //************ KRAJ PROVERE DA LI JE UNESENO DOVOLJNO ILI PREKO BROJEVA OD ONOGA STO TREBA******************
	   
	  /*
	  for(i = 2 ; i <= prava_dim_m ; i = i+1){ // proverava da li su svi redovi i kolone iste tj. da li se svi znakovi ; nalaze na istim udaljenostima jedni od drugih
		  	  
	  if(buff[prava_dim_m + prava_dim_p]!=buff[i*prava_dim_p + i*prava_dim_m + i-1]){
		  
		  printk(KERN_WARNING "VGA_write: los unos tj. pogresan\n");
		  goto skoro_zavrsetak;
		  
		}
	  }
	  */
	  
	  if((prava_dim_p == ioread32(vm->base_addr + 16)) && ((prava_dim_m==ioread32(vm->base_addr + 12)))) // proverava da li se dimenzije poklapaju sa onim iz matmul
	  {
		  
	  for(i = 0 ; i < koliko_unetih_brojeva_b ; i = i+1){
		  
		  upis = uneti_brojevi_b[i];
		  iowrite32(upis, vb->base_addr + 4*i);
		  printk(KERN_WARNING "VGA_write: upisano %u u vb->base_addr + 4*%u\n",upis, i);
		  
		  } 
		  
		  //*******prebacujemo jednodimenzionalni niz u dvodimenzionalni niz tj. matricu*****************************************
			u = 0;
			for( i = 0 ; i < prava_dim_m ; i = i + 1){
				
					for(k = 0 ; k < prava_dim_p ; k = k + 1){
						
						uneti_brojevi_b_matrix[i][k] = uneti_brojevi_b[u];
						u = u + 1;
						printk(KERN_WARNING "VGA_write: uneti_brojevi_b_matrix[%u][%u] = %u\n",i, k, uneti_brojevi_b_matrix[i][k]);
						
					}
			}
			//*************kraj prebacivanja u matricu***********************************************************************
	  printk(KERN_WARNING "VGA_write: poklapaju se n,m i p iz matmul i ovi uneti");
	  goto zavrsetak; 
	  }
	  else{
	    printk(KERN_WARNING "VGA_write: ne poklapaju se n,m i/ili p iz matmul i ovi uneti");
	    goto skoro_zavrsetak; 
	  }
	  
  //----------------------------------------------------------------------------------------------
  
  } //ovo je kraj if(minor == 1) tj. ako upisujemo u bram_b
  
  if(minor == 2){
	  
	  brojac = 0;
	  prava_dim_p = 0;
	  dimenzija_p = strchr(buff, ';'); // trazi prvo pojavljivanje znaka ; u stringu buff i taj broj smesta u dimenzija_p
	  //printk(KERN_WARNING "VGA_write: dimenzija_p = %u\n",dimenzija_p); // ovo daje samo adresu jer je pokazivac
	  
	  razlika_adresa = dimenzija_p - buff;
	  
	  //printk(KERN_WARNING "VGA_write: razlika_adresa = %d\n",razlika_adresa); //ovo je razlika adresa za potrebe debugovanja
	  
	  for( i=0; i<razlika_adresa; i=i+1 ){
		  
		  if(buff[i]==','){
			  prava_dim_p = prava_dim_p + 1;
		  }
		  
	  }
	  prava_dim_p = prava_dim_p + 1; // treba da bi se dobila tacna vrednost jer je na kraju reda uvek ; a ne ,
	  printk(KERN_WARNING "VGA_write: dimenzija_p = %d\n",prava_dim_p); //ovo je dimenzija dobijena iz unosa (ne iz matmula)
	  
	  for( i=0; i<length; i=i+1 ){
		  
		  if(buff[i]==';'){
			  prava_dim_n = prava_dim_n + 1;
		  }
		  
	  }
	  printk(KERN_WARNING "VGA_write: dimenzija_n = %u\n",prava_dim_n); //ovo je dimenzija dobijena iz unosa (ne iz matmula)
	  //printk(KERN_WARNING "VGA_write: Duzina buff = %d\n",length);
	  
	  
	  for(i=0 ; i<length; i=i+1){  //sluzi da smesti pozicije zareza
		  
		  if((buff[i]==',') || (buff[i]==';')){
			  pozicija_zareza[i] = i;
			  
		  }
		  else{
			  pozicija_zareza[i] = 0;
		  }
		  
	  }
	  
	  
	  for(i=0 ; i<length; i=i+1){ // sluzi da ispise pozicije zareza za potrebe debugovanja
	  
		//printk(KERN_WARNING "VGA_write: pozicija_zareza[%u] = %u\n",i, pozicija_zareza[i]);
	  
	  }
	  //**********FUNKCIJA ZA RACUNANJE BROJA CIFARA I KOJE SU TO CIFRE***********************
	  i = length - 1;
	  u = 0; // brojac za niz uneti_brojevi_b
	  while(i>=0){
		  //printk(KERN_WARNING "VGA_write: tek poceo i = %d\n",i);
		if(pozicija_zareza[i]!=0){
			//printk(KERN_WARNING "VGA_write: jeste i razlicito od 0");
			
			veci_indeks = pozicija_zareza[i];
			//printk(KERN_WARNING "VGA_write: veci indeks je = %u", veci_indeks);
			
			for(k = i-1; k >= 0; k = k - 1){
				//printk(KERN_WARNING "VGA_write: usao u petlju sa k = %d", k);
				if((pozicija_zareza[k]!=0) || (k==0)){
					manji_indeks = pozicija_zareza[k];
					//printk(KERN_WARNING "VGA_write: manji indeks je = %u", manji_indeks);
					if(k==0){
						koliko_cifara = veci_indeks-manji_indeks;
					}
					else{
					koliko_cifara = veci_indeks-manji_indeks-1;
					}
					//printk(KERN_WARNING "VGA_write: koliko cifara je = %u", koliko_cifara);
					
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
					//printk(KERN_WARNING "VGA_write: broj_ceo = %lu\n",broj_ceo);
					/*if(broj_ceo > 4095){
						
						printk(KERN_WARNING "VGA_write: Neki od unetih brojeva prelazi 4095, upis se stopira.\n");
						goto skoro_zavrsetak;
						
					}*/ //za bram_c nije potrebno ogranicenje unosa do 4095 po uslovu projekta
					uneti_brojevi_c_reverse[u]=broj_ceo;
					u = u + 1;
					koliko_unetih_brojeva_c = u;
					break;
					
				}
				//break;
			}
		
		}
			if(i == 0){
				break;
			}
		 i = i - 1; 
		 //printk(KERN_WARNING "VGA_write: tek zavrsavam i = %d\n",i);
	  }
	  
	  
	  for(i=0 ; i<u; i=i+1){ // sluzi da napravi non reverse raspored i ispise unos_brojeva_b za potrebe debugovanja
		
		uneti_brojevi_c[i] = uneti_brojevi_c_reverse[koliko_unetih_brojeva_c-i-1];
	    //printk(KERN_WARNING "VGA_write: unos[%u] = %u",i, uneti_brojevi_c[i]);
	  
	  }
	  
	  //*************KRAJ FUNKCIJE ZA RACUNANJE BROJA CIFARA I KOJE SU TO CIFRE****************************
	  
	  //************PROVERA DA LI JE UNESENO DOVOLJNO ILI PREKO BROJEVA OD ONOGA STO TREBA******************
	  
	  
	  if(((koliko_unetih_brojeva_c%prava_dim_p) != 0) || ((koliko_unetih_brojeva_c%prava_dim_n) != 0)){
		  
		  //ovaj if kaze da ako je broj uetih brojeva mora biti broj kolone*broj reda
		  //printk(KERN_WARNING "VGA_write: Uneo si ili vise ili manje brojeva nego sto treba.\n");
		  goto skoro_zavrsetak;
	  }
	  
	   //************ KRAJ PROVERE DA LI JE UNESENO DOVOLJNO ILI PREKO BROJEVA OD ONOGA STO TREBA******************
	   
	  /*
	  for(i = 2 ; i <= prava_dim_m ; i = i+1){ // proverava da li su svi redovi i kolone iste tj. da li se svi znakovi ; nalaze na istim udaljenostima jedni od drugih
		  	  
	  if(buff[prava_dim_m + prava_dim_p]!=buff[i*prava_dim_p + i*prava_dim_m + i-1]){
		  
		  printk(KERN_WARNING "VGA_write: los unos tj. pogresan\n");
		  goto skoro_zavrsetak;
		  
		}
	  }
	  */
	  
	  if((prava_dim_p == ioread32(vm->base_addr + 16)) && ((prava_dim_n==ioread32(vm->base_addr + 8)))) // proverava da li se dimenzije poklapaju sa onim iz matmul
	  {
		  
	  for(i = 0 ; i < koliko_unetih_brojeva_c ; i = i+1){
		  
		  upis = uneti_brojevi_c[i];
		  iowrite32(upis, vc->base_addr + 4*i);
		  printk(KERN_WARNING "VGA_write: upisano %u u vc->base_addr + 4*%u\n",upis, i);
		  
		  } 
		  
		  //*******prebacujemo jednodimenzionalni niz u dvodimenzionalni niz tj. matricu*****************************************
			u = 0;
			for( i = 0 ; i < prava_dim_n ; i = i + 1){
				
					for(k = 0 ; k < prava_dim_p ; k = k + 1){
						
						uneti_brojevi_c_matrix[i][k] = uneti_brojevi_c[u];
						u = u + 1;
						printk(KERN_WARNING "VGA_write: uneti_brojevi_c_matrix[%u][%u] = %u\n",i, k, uneti_brojevi_c_matrix[i][k]);
						
					}
			}
			//*************kraj prebacivanja u matricu***********************************************************************
	  printk(KERN_WARNING "VGA_write: poklapaju se n,m i p iz matmul i ovi uneti");
	  goto zavrsetak; 
	  }
	  else{
	    printk(KERN_WARNING "VGA_write: ne poklapaju se n,m i/ili p iz matmul i ovi uneti");
	    goto skoro_zavrsetak; 
	  }
  
  } //ovo je kraj if(minor == 2) tj. ako upisujemo u bram_c
  
  //----------------------------------------------------------------------------------------------
  
  skoro_zavrsetak:
  printk(KERN_WARNING "VGA_write: pogresan unos");
  zavrsetak: 
  return length;

  
} // ovo je kraj write funkcije

//***************************************************
// HELPER FUNCTIONS (STRING TO INTEGER)


//***************************************************
// INIT AND EXIT FUNCTIONS OF THE DRIVER
//napravljene da prave 2 node fajla

static int __init matrix_multiplier_init(void)
{

  int ret = 0;
  int_cnt = 0;
  //char buff[10];

  printk(KERN_INFO "matrix_multiplier_init: Initialize Module \"%s\"\n", DEVICE_NAME);
  ret = alloc_chrdev_region(&my_dev_id, 0, 4, "matrix_multiplier_region");
  if (ret)
  {
    printk(KERN_ALERT "<1>Failed CHRDEV!.\n");
    return -1;
  }
  printk(KERN_INFO "Succ CHRDEV!.\n");
  my_class = class_create(THIS_MODULE, "matrix_multiplier_drv");
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
    printk(KERN_ERR "matrix_multiplier_init: Failed to add cdev\n");
    goto fail_2;
  }
  printk(KERN_INFO "matrix_multiplier device init.\n");

  return platform_driver_register(&matrix_multiplier_driver);

 fail_2:
  device_destroy(my_class, my_dev_id );
 fail_1:
  class_destroy(my_class);
 fail_0:
  unregister_chrdev_region(my_dev_id, 1);
  return -1;

} 

static void __exit matrix_multiplier_exit(void)  		
{

  platform_driver_unregister(&matrix_multiplier_driver);
  cdev_del(my_cdev);
  device_destroy(my_class, MKDEV(MAJOR(my_dev_id),0));
  device_destroy(my_class, MKDEV(MAJOR(my_dev_id),1));
  device_destroy(my_class, MKDEV(MAJOR(my_dev_id),2));
  device_destroy(my_class, MKDEV(MAJOR(my_dev_id),3));
  class_destroy(my_class);
  unregister_chrdev_region(my_dev_id, 4);
  printk(KERN_INFO "matrix_multiplier_exit: Exit Device Module \"%s\".\n", DEVICE_NAME);
}

module_init(matrix_multiplier_init);
module_exit(matrix_multiplier_exit);

MODULE_AUTHOR ("FTN");
MODULE_DESCRIPTION("Test Driver for VGA output.");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("custom:vga");
