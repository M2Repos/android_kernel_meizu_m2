#include <linux/fs.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#define TORCH_INFO(fmt, arg...)   pr_info("torch_sysfs: "fmt"\n", ##arg)
#define TORCH_ERROR(fmt, arg...)  pr_err("torch_sysfs: "fmt"\n", ##arg)

static struct proc_dir_entry *torch_proc_entry = NULL;

static int torch_status = 0;
static char* proc_name = "torch";

extern int FL_Disable(void);
extern int FL_Enable(void);
extern int FL_Init(void);

static void change_torch_status(int enable) {
	torch_status = enable;

	if (enable) {
		FL_Init();
		FL_Enable();
	} else
		FL_Disable();
}

static ssize_t torch_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data) {
	int value;
	char data_buf[50] = {0};

	if ((NULL == filp) || (NULL == buff))
		return -1;

	if ((len <= 0) || (len > 50))
		return -1;

	if (copy_from_user(data_buf, buff, len))
		return -EFAULT;

	value = simple_strtoul(data_buf, NULL, 16);
	if (value != 0 && value != 1) {
		TORCH_ERROR("%s invalid value.\n", __func__);
		return -1;
	} else
		change_torch_status(value);

	TORCH_INFO("%s status changed to %d.", __func__, value);

	return len;
}

static int torch_proc_show(struct seq_file *m, void *v) {
	seq_printf(m, "%d\n", torch_status);
	return 0;
}

static int torch_proc_open(struct inode *inode, struct file *file) {
	return single_open(file, torch_proc_show, NULL);
}

static const struct file_operations torch_proc_fops = {
	.open = torch_proc_open,
	.write = torch_proc_write,
	.read = seq_read,
};

static int __init torch_init(void) {
	torch_proc_entry = proc_create(proc_name, 0664, NULL, &torch_proc_fops);
	if (torch_proc_entry == NULL)
		TORCH_ERROR("proc_create %s failed!", proc_name);

	return 0;
}

static void __exit torch_exit(void) {
	if (torch_proc_entry != NULL) {
		remove_proc_entry(proc_name, NULL);
		torch_proc_entry = NULL;
	}
}

module_init(torch_init);
module_exit(torch_exit);
