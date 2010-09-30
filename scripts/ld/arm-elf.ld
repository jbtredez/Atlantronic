/*
script d'édition des liens

point d'entrée : isr_reset
section .text en flash : code
section .data en flash et en ram : initialisation des constantes en ram à partir de la flash par isr_reset (isr.c)
section .bss en ram : choses à initialiser à 0 (fait par isr_reset dans isr.c)
*/


/* point d'entrée */
ENTRY(isr_reset)

/* zones mémoires : TODO à modifier FLASH 256K, SRAM 64K. Ce sont les valeurs pour la machine émulée sous qemu (trouver comment modifier cela sous qemu) */
MEMORY
{
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 64K
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 8K
}

/* placement du haut de la stack : tout au bout de la SRAM */
_stack = 0x20000000 + 8K;

SECTIONS
{
	/* Section .text
	   La section débute par le vecteur d'interruption (section .isr_vector, fichier isr.c).
	   La section doit impérativement débuter en 0x00 pour placer le vecteur au bon endroit
	*/
	.text : AT (0x00)
	{
		KEEP(*(.isr_vector))
		*(.text)
		*(.init)
		*(.rodata*)
		*(.fini)
		*(.eh_frame)
		__text_end__ = .;
	} > FLASH

	/* Section .data
	   Exécution en ram, au début sur la flash. On fait la copie au lancement dans la fonction isr_reset
	   En SRAM, on va de __data_start__ à __data_end__. Comme la section en FLASH est mise directement après .text,
	   on débute la copie à partir de __test_end__
	   Ne pas mettre de section entre la fin de .text et le début de .data
	 */
	.data :
	{
		__data_start__ = .;
		*(vtable)
		*(.data)

		/* tableau (trié) des fonctions d'init */
		__initcall_start = .;
		*(.initcall.init)
		*(SORT(.initcall.init.*))
		__initcall_end = .;

		__data_end__ = .;
	} > FLASH

	/* Section .bss :
	   Données à mettre à zéro. C'est réalisé dans la fonction isr_reset (fichier isr.c)
	*/
	.bss :
	{
		__bss_start__ = .;
		*(.bss)
		*(COMMON)
		__bss_end__ = .;
	} > SRAM

	PROVIDE (end = .);
}
