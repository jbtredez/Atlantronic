/*
script d'édition des liens

point d'entrée : isr_reset
section .text en flash : code
section .data en flash et en ram : initialisation des constantes en ram à partir de la flash par isr_reset (isr.c)
section .bss en ram : choses à initialiser à 0 (fait par isr_reset dans isr.c)
*/


/* point d'entrée */
ENTRY(isr_reset)

MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 256K
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}

/* placement du haut de la stack : tout au bout de la SRAM */
_ram_top = 0x20000000 + 64K;

SECTIONS
{
	/* Section .text
	   La section débute par le vecteur d'interruption (section .isr_vector, fichier isr.c).
	   La section doit impérativement débuter en 0x00 pour placer le vecteur au bon endroit
	*/
	.text : AT (0x08000000)
	{
		KEEP(*(.isr_vector))
		*(.text)
		/* tableau (trié) des fonctions d'init */
		__initcall_start = .;
		*(.initcall.init)
		*(SORT(.initcall.init.*))
		__initcall_end = .;

		/* tableau (trié) des fonctions d'exit */
		__exitcall_start = .;
		*(.exitcall.exit)
		*(SORT(.exitcall.exit.*))
		__exitcall_end = .;

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
		__data_end__ = .;
	} > SRAM AT > FLASH

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

	_stack_top = _ram_top - 16;

	PROVIDE (end = .);
}
