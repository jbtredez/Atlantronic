#set prompt \033[01;32m(gdb) \033[0m

define mri
	monitor reset init
end

define prog
	monitor reset init
	load
	monitor reset init
end

define ptasks
	_ptasks 0
end

define ptasks_stack
	_ptasks 1
end

define pt
	p *((tskTCB *) $arg0)
end

# arg0 : adresse tcb
# arg1 : full ou pas
define ps
	set $stack = ((tskTCB*)$arg0)->pxTopOfStack
	set $j = 0
	while $j < ((tskTCB*)$arg0)->pxStackBegin - $stack
		if $stack[$j] > 0x8000000 && $stack[$j] < &_etext
			list *($stack[$j]),1
		else
			if $argc == 2
				if $arg1 == 1
					printf "%10x\n",  $stack[$j]
				end
			end
		end
		set $j++
	end
end

define _ptasks
	printf "Il y a %i taches\n", uxCurrentNumberOfTasks
	set $match_time = 0
	set $systime = systick_time.ms / 1000.0f + (SysTick->LOAD - SysTick->VAL)/((double)SysTick->LOAD) / 1000.0f
	set $start_match_time = systick_time_start_match.ms/1000.0f + systick_time_start_match.ns/1000000000.0f

	if $start_match_time > 0
		set $match_time = $systime - $start_match_time 
	end

	set $static_alloc = (unsigned int)&_ebss - (unsigned int)&_sdata - sizeof(ucHeap)
	set $malloc_percent = 100.0f * xNextFreeByte / (float)sizeof(ucHeap)
	set $ram_size = (unsigned int)&_estack - (unsigned int)&_sdata
	printf "static alloc : %5i\n", $static_alloc
	printf "malloc       : %5i / %5i (%6.2f%%)\n", xNextFreeByte, sizeof(ucHeap), $malloc_percent
	printf "main stack   : %5i\n", $ram_size - $static_alloc - sizeof(ucHeap)
	printf "uptime       : %10.6f\n", $systime
	printf "match_time   : %10.6f\n", $match_time

	echo \033[01;32m
	printf "  R   "
	if pxCurrentTCB != 0
		printf "%s\n\n", pxCurrentTCB->pcTaskName
	else
		printf "aucune\n\n"
	end
	echo \033[0m

	echo \033[01;34m
	if $arg0
		printf " etat | delai (ms) |    nom   |   addr     |  time used  |   cpu   | free stack (32 bit)  |\n"
		printf "      |            |          |            |             |    %%    | current   |   min    |\n"
	else
		printf " etat | delai (ms) |    nom   |   addr     |  time used  |   cpu   | free stack|\n"
		printf "      |            |          |            |             |    %%    | current   |\n"
	end

	echo \033[0m
	set $nb_pri = sizeof pxReadyTasksLists / sizeof pxReadyTasksLists[0]
	set $i = $nb_pri - 1
	
	while $i >= 0
		set $n = pxReadyTasksLists[$i].uxNumberOfItems
		set $list_end = &pxReadyTasksLists[$i].xListEnd
		set $list_next = pxReadyTasksLists[$i].xListEnd.pxNext

		while $list_next != $list_end && $n > 0
			printf "  P%2i |          0 |", $i
			pTcb ((tskTCB*)$list_next->pvOwner) $arg0
			set $list_next = $list_next.pxNext
			set $n--
		end
		set $i--
	end

	set $n = xPendingReadyList.uxNumberOfItems
	set $list_end = &xPendingReadyList.xListEnd
	set $list_next = xPendingReadyList.xListEnd.pxNext

	while $list_next != $list_end && $n > 0
		printf "  PR  |          0 |"
		pTcb ((tskTCB*)$list_next->pvOwner) $arg0
		set $list_next = $list_next.pxNext
		set $n--
	end
	set $i--

	set $n = xDelayedTaskList1.uxNumberOfItems
	set $list_end = &xDelayedTaskList1.xListEnd
	set $list_next = xDelayedTaskList1.xListEnd.pxNext

	while $list_next != $list_end && $n > 0
		printf "  D1  | %10.2f |", ($list_next->xItemValue - $systime*1000.0f)
		pTcb ((tskTCB*)$list_next->pvOwner) $arg0
		set $list_next = $list_next.pxNext
		set $n--
	end
	set $i--

	set $n = xDelayedTaskList2.uxNumberOfItems
	set $list_end = &xDelayedTaskList2.xListEnd
	set $list_next = xDelayedTaskList2.xListEnd.pxNext

	while $list_next != $list_end && $n > 0
		printf "  D2  | %10.2f |", ($list_next->xItemValue - $systime*1000.0f)
		pTcb ((tskTCB*)$list_next->pvOwner) $arg0
		set $list_next = $list_next.pxNext
		set $n--
	end
	set $i--

	set $n = xSuspendedTaskList.uxNumberOfItems
	set $list_end = &xSuspendedTaskList.xListEnd
	set $list_next = xSuspendedTaskList.xListEnd.pxNext

	while $list_next != $list_end && $n > 0
		printf "  S   |        inf |"
		pTcb ((tskTCB*)$list_next->pvOwner) $arg0
		set $list_next = $list_next.pxNext
		set $n--
	end
	set $i--

end

define pTcb
	printf " %8s | %10p | %11.6f | %7.3f | %9i |", $arg0->pcTaskName, $arg0, $arg0->ulRunTimeCounter.ms/1000.0f + $arg0->ulRunTimeCounter.ns/1000000000.0f, ($arg0->ulRunTimeCounter.ms/1000.0f + $arg0->ulRunTimeCounter.ns/1000000000.0f)/(ulTotalRunTime.ms/1000.0f + ulTotalRunTime.ns/1000000000.0f)*100, $arg0->pxTopOfStack - $arg0->pxStack

	if $arg1
		set $stack_min_free = 0
		while $arg0->pxStack[$stack_min_free] == 0xa5a5a5a5
			set $stack_min_free++
		end
		printf " %8i |\n", $stack_min_free
	else
		printf "\n"
	end
end
